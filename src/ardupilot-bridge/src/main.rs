mod ardupilot_ipc;
mod can_output;
mod config;
mod coordinate;
mod elodin;
mod hitl;

use crate::ardupilot_ipc::{ServoOutput, SitlJsonPacket, ImuData};
use crate::can_output::CanOutput;
use crate::config::{Config, HomeLocation};
use crate::coordinate::{gyro_flu_to_frd, accel_flu_to_frd, ubx_lla_to_ned, ubx_vel_to_ms, ubx_heading_to_rad};
use crate::elodin::{M10QInput, MotorTelemetry, SensorInput};

use anyhow::Context;
use clap::Parser;
use impeller2::types::PacketId;
use impeller2_stellar::{Client, SinkExt, StreamExt};
use std::net::{SocketAddr, UdpSocket};
use std::sync::{Arc, Mutex};
use std::time::Duration;

fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt::init();
    stellarator::run(run)
}

/// Cached GPS state, updated by the GPS subscription task and read by the
/// IMU-rate main loop. GPS arrives at ~2.5 Hz; IMU at 400+ Hz.
#[derive(Debug, Clone, Copy)]
struct GpsCache {
    position_ned: [f64; 3],
    velocity_ned: [f64; 3],
    yaw_rad: f64,
    has_fix: bool,
    satellites: u8,
    h_acc_m: f64,
}

impl Default for GpsCache {
    fn default() -> Self {
        Self {
            position_ned: [0.0; 3],
            velocity_ned: [0.0; 3],
            yaw_rad: 0.0,
            has_fix: false,
            satellites: 0,
            h_acc_m: 999.0,
        }
    }
}

async fn run() -> anyhow::Result<()> {
    let config = Config::parse();
    tracing::info!("ardupilot-bridge starting with config: {:?}", config);

    let home = HomeLocation::parse(&config.home)
        .context("parse --home / AP_HOME")?;
    tracing::info!(
        "home location: lat={:.6} lon={:.6} alt={:.1}m",
        home.lat_deg, home.lon_deg, home.alt_m,
    );

    let telemetry_id: PacketId = fastrand::u16(..).to_le_bytes();

    loop {
        if let Err(err) = bridge_loop(&config, &home, telemetry_id).await {
            tracing::error!("bridge error: {:?}", err);
            stellarator::sleep(Duration::from_millis(500)).await;
        }
    }
}

async fn bridge_loop(
    config: &Config,
    home: &HomeLocation,
    telemetry_id: PacketId,
) -> anyhow::Result<()> {
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

    // -----------------------------------------------------------------------
    // GPS subscription -- runs in a background thread, updates shared cache.
    // The M10Q vtable arrives at ~2.5 Hz from the STM32 UBX parser.
    // -----------------------------------------------------------------------
    let gps_cache = Arc::new(Mutex::new(GpsCache::default()));
    {
        let gps_cache = Arc::clone(&gps_cache);
        let home = *home;
        let elodin_addr_str = config.elodin_addr.clone();
        std::thread::spawn(move || {
            if let Err(e) = stellarator::run(move || gps_task(elodin_addr_str, home, gps_cache)) {
                tracing::error!("GPS task fatal: {:?}", e);
            }
        });
    }

    // -----------------------------------------------------------------------
    // Main sensor loop -- IMU-rate, drives ArduPilot JSON packets.
    // -----------------------------------------------------------------------
    let mut sub = client.subscribe::<SensorInput>().await?;
    let mut tick: u64 = 0;
    let start = std::time::Instant::now();

    loop {
        // Check for servo output from ArduPilot (non-blocking)
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
                    let gps = gps_cache.lock().unwrap();
                    tracing::info!(
                        "tick={} motors=[{:.2},{:.2},{:.2},{:.2}] gps_fix={} sats={} h_acc={:.2}m",
                        tick,
                        motor_cmd[0], motor_cmd[1], motor_cmd[2], motor_cmd[3],
                        gps.has_fix, gps.satellites, gps.h_acc_m,
                    );
                }
            }
        }

        // Read IMU sensor data from Elodin-DB and send to ArduPilot
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

            let gps = gps_cache.lock().unwrap();
            let (position, velocity, attitude) = if gps.has_fix {
                (
                    gps.position_ned,
                    gps.velocity_ned,
                    [0.0, 0.0, gps.yaw_rad],
                )
            } else {
                ([0.0; 3], [0.0; 3], [0.0; 3])
            };
            drop(gps);

            let packet = SitlJsonPacket {
                timestamp,
                imu: ImuData {
                    gyro: gyro_frd,
                    accel_body: accel_frd,
                },
                position,
                velocity,
                attitude,
            };

            let json_bytes = packet.to_json_bytes();
            udp_socket
                .send_to(&json_bytes, target)
                .context("send sensor JSON to ArduPilot")?;
        }
    }
}

/// Background task: subscribes to the M10Q GPS vtable and updates the shared
/// cache. Reconnects on error. The cache is read by the IMU-rate main loop.
///
/// TODO: simulation -- when running in HITL/sim-hitl mode the M10Q vtable
/// won't exist. This task will keep retrying silently. A future sim GPS
/// synthesizer should write synthetic M10Q rows to the DB so this same
/// code path works for both real hardware and simulation.
async fn gps_task(
    elodin_addr: String,
    home: HomeLocation,
    cache: Arc<Mutex<GpsCache>>,
) -> anyhow::Result<()> {
    loop {
        match gps_subscribe_loop(&elodin_addr, &home, &cache).await {
            Ok(()) => {}
            Err(e) => {
                tracing::warn!("GPS subscription error (will retry): {}", e);
                stellarator::sleep(Duration::from_secs(2)).await;
            }
        }
    }
}

async fn gps_subscribe_loop(
    elodin_addr: &str,
    home: &HomeLocation,
    cache: &Arc<Mutex<GpsCache>>,
) -> anyhow::Result<()> {
    let addr: SocketAddr = elodin_addr.parse().context("parse elodin addr for GPS")?;
    let mut client = Client::connect(addr).await.map_err(anyhow::Error::from)?;
    let mut sub = client.subscribe::<M10QInput>().await?;
    tracing::info!("GPS: subscribed to M10Q vtable");

    let mut gps_tick: u64 = 0;
    loop {
        let gps = sub.next().await?;
        let has_fix = gps.fix_type >= 3;

        let position_ned = if has_fix {
            ubx_lla_to_ned(
                gps.lat, gps.lon, gps.alt_msl,
                home.lat_deg, home.lon_deg, home.alt_m,
            )
        } else {
            [0.0; 3]
        };

        let velocity_ned = if has_fix {
            ubx_vel_to_ms(gps.vel_ned)
        } else {
            [0.0; 3]
        };

        let yaw_rad = ubx_heading_to_rad(gps.heading_motion);
        let h_acc_m = gps.h_acc as f64 * 1e-3;

        {
            let mut c = cache.lock().unwrap();
            c.position_ned = position_ned;
            c.velocity_ned = velocity_ned;
            c.yaw_rad = yaw_rad;
            c.has_fix = has_fix;
            c.satellites = gps.satellites;
            c.h_acc_m = h_acc_m;
        }

        gps_tick += 1;
        if gps_tick % 10 == 1 {
            tracing::info!(
                "GPS: fix={} sats={} lat={:.7} lon={:.7} alt={:.1}m h_acc={:.2}m NED=[{:.1},{:.1},{:.1}]",
                gps.fix_type,
                gps.satellites,
                gps.lat as f64 * 1e-7,
                gps.lon as f64 * 1e-7,
                gps.alt_msl as f64 * 1e-3,
                h_acc_m,
                position_ned[0], position_ned[1], position_ned[2],
            );
        }
    }
}
