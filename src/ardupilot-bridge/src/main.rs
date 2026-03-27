mod ardupilot_ipc;
mod can_output;
mod config;
mod coordinate;
mod elodin;
mod hitl;

use crate::ardupilot_ipc::{ServoOutput, SitlJsonPacket, ImuData};
use crate::can_output::CanOutput;
use crate::config::Config;
use crate::coordinate::{gyro_flu_to_frd, accel_flu_to_frd};
use crate::elodin::{MotorTelemetry, SensorInput};

use anyhow::Context;
use clap::Parser;
use impeller2::types::PacketId;
use impeller2_stellar::{Client, SinkExt, StreamExt};
use std::net::{SocketAddr, UdpSocket};
use std::time::Duration;

fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt::init();
    stellarator::run(run)
}

async fn run() -> anyhow::Result<()> {
    let config = Config::parse();
    tracing::info!("ardupilot-bridge starting with config: {:?}", config);

    // Stable VTable ID so retries don't spam the DB with new registrations.
    let telemetry_id: PacketId = fastrand::u16(..).to_le_bytes();

    loop {
        if let Err(err) = bridge_loop(&config, telemetry_id).await {
            tracing::error!("bridge error: {:?}", err);
            stellarator::sleep(Duration::from_millis(500)).await;
        }
    }
}

async fn bridge_loop(config: &Config, telemetry_id: PacketId) -> anyhow::Result<()> {
    let elodin_addr: SocketAddr = config
        .elodin_addr
        .parse()
        .context("invalid elodin_addr")?;

    tracing::info!("connecting to Elodin-DB at {}", elodin_addr);
    let mut client = Client::connect(elodin_addr)
        .await
        .map_err(anyhow::Error::from)?;

    client.init_world::<MotorTelemetry>(telemetry_id).await?;

    // ArduPilot SITL JSON protocol:
    //  1. Bridge binds UDP to the control port (default 9002)
    //  2. ArduPilot sends servo output TO this port
    //  3. Bridge receives servo packet, notes ArduPilot's source address
    //  4. Bridge sends JSON sensor data back TO ArduPilot's address
    //
    // On first iteration before we know ArduPilot's address, we send to
    // localhost on the control port to trigger the initial handshake.
    let bind_addr: SocketAddr = format!("0.0.0.0:{}", config.servo_port)
        .parse()
        .unwrap();

    let udp_socket = UdpSocket::bind(bind_addr)
        .context("bind UDP control port")?;
    udp_socket
        .set_read_timeout(Some(Duration::from_millis(100)))
        .ok();

    let mut can = CanOutput::new(&config.can_interface);
    can.open().context("open CAN interface")?;

    tracing::info!(
        "bridge bound to UDP :{}, waiting for ArduPilot servo output, CAN={}",
        config.servo_port,
        if can.is_enabled() { &config.can_interface } else { "disabled" }
    );

    let mut ap_addr: Option<SocketAddr> = None;

    // Start HITL TCP server in a background thread if enabled.
    // The HITL server shares the UDP socket and ap_addr with the main loop
    // via Arc<Mutex<>>. When a HITL client connects, it drives ArduPilot
    // directly from the TCP connection's FDM packets.
    if config.hitl_port > 0 {
        let hitl_port = config.hitl_port;
        let hitl_udp = udp_socket.try_clone().context("clone UDP for HITL")?;
        std::thread::spawn(move || {
            let mut hitl_ap_addr: Option<SocketAddr> = None;
            if let Err(e) = hitl::run_hitl_loop(hitl_port, &hitl_udp, &mut hitl_ap_addr) {
                tracing::error!("HITL server error: {}", e);
            }
        });
    }

    let mut sub = client.subscribe::<SensorInput>().await?;
    let mut tick: u64 = 0;
    let start = std::time::Instant::now();

    loop {
        // First: check for servo output from ArduPilot (non-blocking)
        let mut servo_buf = [0u8; 256];
        if let Ok((n, src)) = udp_socket.recv_from(&mut servo_buf) {
            if let Some(servo) = ServoOutput::from_bytes(&servo_buf[..n]) {
                if ap_addr.is_none() {
                    tracing::info!("ArduPilot connected from {}", src);
                }
                ap_addr = Some(src);

                let motors_norm = servo.motors_normalized(config.num_motors);
                let motor_pwm: [u16; 4] = [
                    servo.pwm[0],
                    servo.pwm[1],
                    servo.pwm[2],
                    servo.pwm[3],
                ];
                let motor_cmd: [f32; 4] = [
                    motors_norm.get(0).copied().unwrap_or(0.0) as f32,
                    motors_norm.get(1).copied().unwrap_or(0.0) as f32,
                    motors_norm.get(2).copied().unwrap_or(0.0) as f32,
                    motors_norm.get(3).copied().unwrap_or(0.0) as f32,
                ];

                if can.is_enabled() {
                    if let Err(e) = can.send_esc_command(&motors_norm) {
                        tracing::warn!("CAN send error: {}", e);
                    }
                }

                let telemetry = MotorTelemetry::new(motor_pwm, motor_cmd);
                let table = telemetry.to_table_packet(telemetry_id);
                sub.send(table).await.0?;

                tick += 1;
                if tick % 1000 == 0 {
                    tracing::info!(
                        "tick={} motors=[{:.2}, {:.2}, {:.2}, {:.2}]",
                        tick,
                        motor_cmd[0],
                        motor_cmd[1],
                        motor_cmd[2],
                        motor_cmd[3]
                    );
                }
            }
        }

        // Then: read sensor data from Elodin-DB and send to ArduPilot
        if let Some(target) = ap_addr {
            let input = sub.next().await?;
            let timestamp = start.elapsed().as_secs_f64();

            let gyro_frd = gyro_flu_to_frd([
                input.gyro[0] as f64,
                input.gyro[1] as f64,
                input.gyro[2] as f64,
            ]);
            let accel_frd = accel_flu_to_frd([
                input.accel[0] as f64,
                input.accel[1] as f64,
                input.accel[2] as f64,
            ]);

            let packet = SitlJsonPacket {
                timestamp,
                imu: ImuData {
                    gyro: gyro_frd,
                    accel_body: accel_frd,
                },
                position: [0.0, 0.0, 0.0],
                velocity: [0.0, 0.0, 0.0],
                attitude: [0.0, 0.0, 0.0],
            };

            let json_bytes = packet.to_json_bytes();
            udp_socket
                .send_to(&json_bytes, target)
                .context("send sensor JSON to ArduPilot")?;
        }
    }
}
