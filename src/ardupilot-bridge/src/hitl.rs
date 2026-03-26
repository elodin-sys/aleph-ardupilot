/// HITL (Hardware-In-The-Loop) socket interface.
///
/// When enabled, accepts a TCP connection from a simulation script.
/// The simulation sends FDM sensor packets and receives motor command
/// packets, matching the Betaflight SITL post_step pattern.
///
/// Protocol (binary, little-endian):
///
/// Sim -> Bridge (FDM packet, 144 bytes = 18 x f64):
///   timestamp(1) + gyro_rpy(3) + accel_xyz(3) + quat_wxyz(4) +
///   vel_xyz(3) + pos_xyz(3) + pressure(1)
///
/// Bridge -> Sim (Motor packet, 32 bytes = 4 x f64):
///   motor[0..4] normalized [0.0, 1.0]
use std::io::{Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::time::Duration;

use crate::ardupilot_ipc::{ImuData, ServoOutput, SitlJsonPacket};

pub const FDM_PACKET_SIZE: usize = 18 * 8;
pub const MOTOR_PACKET_SIZE: usize = 4 * 8;

#[derive(Debug)]
pub struct FdmData {
    pub timestamp: f64,
    pub gyro: [f64; 3],
    pub accel: [f64; 3],
    pub quat: [f64; 4],
    pub velocity: [f64; 3],
    pub position: [f64; 3],
    pub pressure: f64,
}

pub fn parse_fdm_packet(data: &[u8; FDM_PACKET_SIZE]) -> FdmData {
    let mut vals = [0.0f64; 18];
    for (i, chunk) in data.chunks_exact(8).enumerate() {
        vals[i] = f64::from_le_bytes(chunk.try_into().unwrap());
    }
    FdmData {
        timestamp: vals[0],
        gyro: [vals[1], vals[2], vals[3]],
        accel: [vals[4], vals[5], vals[6]],
        quat: [vals[7], vals[8], vals[9], vals[10]],
        velocity: [vals[11], vals[12], vals[13]],
        position: [vals[14], vals[15], vals[16]],
        pressure: vals[17],
    }
}

pub fn encode_motor_packet(motors: &[f64; 4]) -> [u8; MOTOR_PACKET_SIZE] {
    let mut buf = [0u8; MOTOR_PACKET_SIZE];
    for (i, &m) in motors.iter().enumerate() {
        buf[i * 8..(i + 1) * 8].copy_from_slice(&m.to_le_bytes());
    }
    buf
}

impl FdmData {
    pub fn to_sitl_json(&self) -> SitlJsonPacket {
        let (roll, pitch, yaw) = quat_to_euler(self.quat);
        SitlJsonPacket {
            timestamp: self.timestamp,
            imu: ImuData {
                gyro: self.gyro,
                accel_body: self.accel,
            },
            position: self.position,
            velocity: self.velocity,
            attitude: [roll, pitch, yaw],
        }
    }
}

fn quat_to_euler(q: [f64; 4]) -> (f64, f64, f64) {
    let [w, x, y, z] = q;
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = sinr_cosp.atan2(cosr_cosp);

    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        std::f64::consts::FRAC_PI_2.copysign(sinp)
    } else {
        sinp.asin()
    };

    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = siny_cosp.atan2(cosy_cosp);

    (roll, pitch, yaw)
}

/// Run the HITL server loop. Blocks, accepting one client at a time.
/// Each client connection drives a lockstep loop:
///   1. Read FDM from sim
///   2. Forward as JSON to ArduPilot via UDP
///   3. Read servo response from ArduPilot via UDP
///   4. Send motor packet back to sim
pub fn run_hitl_loop(
    listen_port: u16,
    ap_udp: &std::net::UdpSocket,
    ap_addr: &mut Option<SocketAddr>,
) -> anyhow::Result<()> {
    let listener = TcpListener::bind(SocketAddr::from(([0, 0, 0, 0], listen_port)))?;
    tracing::info!("HITL server listening on :{}", listen_port);

    for stream in listener.incoming() {
        match stream {
            Ok(client) => {
                let peer = client.peer_addr().unwrap_or(SocketAddr::from(([0, 0, 0, 0], 0)));
                tracing::info!("HITL client connected from {}", peer);
                if let Err(e) = handle_hitl_client(client, ap_udp, ap_addr) {
                    tracing::warn!("HITL client {} disconnected: {}", peer, e);
                }
            }
            Err(e) => {
                tracing::warn!("HITL accept error: {}", e);
            }
        }
    }
    Ok(())
}

fn handle_hitl_client(
    mut client: TcpStream,
    ap_udp: &std::net::UdpSocket,
    ap_addr: &mut Option<SocketAddr>,
) -> anyhow::Result<()> {
    client.set_nodelay(true)?;
    ap_udp.set_read_timeout(Some(Duration::from_millis(500)))?;

    let mut fdm_buf = [0u8; FDM_PACKET_SIZE];
    let mut servo_buf = [0u8; 256];
    let mut tick: u64 = 0;

    loop {
        client.read_exact(&mut fdm_buf)?;
        let fdm = parse_fdm_packet(&fdm_buf);
        let json_pkt = fdm.to_sitl_json();
        let json_bytes = json_pkt.to_json_bytes();

        // Send sensor JSON to ArduPilot. If we know AP's address, use it;
        // otherwise send to localhost:9002 to trigger the initial handshake.
        let target = ap_addr.unwrap_or(SocketAddr::from(([127, 0, 0, 1], 9002)));
        ap_udp.send_to(&json_bytes, target)?;

        // Try to receive servo output from ArduPilot
        let motors = match ap_udp.recv_from(&mut servo_buf) {
            Ok((n, src)) => {
                *ap_addr = Some(src);
                if let Some(servo) = ServoOutput::from_bytes(&servo_buf[..n]) {
                    let norm = servo.motors_normalized(4);
                    [
                        norm.get(0).copied().unwrap_or(0.0),
                        norm.get(1).copied().unwrap_or(0.0),
                        norm.get(2).copied().unwrap_or(0.0),
                        norm.get(3).copied().unwrap_or(0.0),
                    ]
                } else {
                    [0.0; 4]
                }
            }
            Err(_) => [0.0; 4],
        };

        let motor_pkt = encode_motor_packet(&motors);
        client.write_all(&motor_pkt)?;

        tick += 1;
        if tick % 1000 == 0 {
            tracing::info!(
                "HITL tick={} motors=[{:.2}, {:.2}, {:.2}, {:.2}]",
                tick,
                motors[0],
                motors[1],
                motors[2],
                motors[3]
            );
        }
    }
}
