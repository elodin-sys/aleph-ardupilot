/// HITL (Hardware-In-The-Loop) socket interface.
///
/// When enabled, accepts a TCP connection from an Elodin simulation's
/// post_step callback. The simulation sends FDM sensor packets and
/// receives motor command packets, matching the Betaflight SITL pattern.
///
/// Protocol (binary, little-endian):
///
/// Sim -> Bridge (FDM packet, 144 bytes = 18 x f64):
///   timestamp(1) + gyro_rpy(3) + accel_xyz(3) + quat_wxyz(4) +
///   vel_xyz(3) + pos_xyz(3) + pressure(1)
///
/// Bridge -> Sim (Motor packet, 32 bytes = 4 x f64):
///   motor[0..4] normalized [0.0, 1.0]

use std::net::SocketAddr;

pub const FDM_PACKET_SIZE: usize = 18 * 8; // 18 f64s = 144 bytes
pub const MOTOR_PACKET_SIZE: usize = 4 * 8; // 4 f64s = 32 bytes

pub struct HitlServer {
    listen_addr: SocketAddr,
    active: bool,
}

impl HitlServer {
    pub fn new(port: u16) -> Option<Self> {
        if port == 0 {
            return None;
        }
        Some(Self {
            listen_addr: SocketAddr::from(([0, 0, 0, 0], port)),
            active: false,
        })
    }

    pub fn is_active(&self) -> bool {
        self.active
    }

    /// Start the HITL TCP listener.
    /// TODO: Phase 4 - Accept connection, replace sensor source with HITL data,
    /// send motor commands back to the sim.
    pub async fn run(&mut self) -> anyhow::Result<()> {
        tracing::info!("HITL server listening on {}", self.listen_addr);
        // TODO: Phase 4 implementation
        // 1. Accept TCP connection
        // 2. Read FDM packets (144 bytes each)
        // 3. Convert to SitlJsonPacket and send to ArduPilot
        // 4. Read servo output from ArduPilot
        // 5. Send motor values back to sim
        Ok(())
    }
}

/// Parse an FDM packet from the simulation.
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

/// Encode motor values into a binary packet for the simulation.
pub fn encode_motor_packet(motors: &[f64; 4]) -> [u8; MOTOR_PACKET_SIZE] {
    let mut buf = [0u8; MOTOR_PACKET_SIZE];
    for (i, &m) in motors.iter().enumerate() {
        buf[i * 8..(i + 1) * 8].copy_from_slice(&m.to_le_bytes());
    }
    buf
}
