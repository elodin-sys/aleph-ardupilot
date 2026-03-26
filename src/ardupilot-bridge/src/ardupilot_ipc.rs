/// ArduPilot SITL JSON interface.
///
/// Protocol:
///   - Bridge -> ArduPilot: JSON sensor data over UDP (default port 9003)
///   - ArduPilot -> Bridge: Binary servo/motor output over UDP (default port 9002)
///
/// JSON format (from ArduPilot SIM_JSON docs):
/// {
///   "timestamp": <seconds>,
///   "imu": { "gyro": [gx, gy, gz], "accel_body": [ax, ay, az] },
///   "position": [north, east, down],
///   "velocity": [vn, ve, vd],
///   "attitude": [roll, pitch, yaw]
/// }
///
/// Servo output binary format:
///   u16 magic (18458)
///   u16 frame_rate
///   u32 frame_count
///   u16[16] pwm_values (microseconds, 1000-2000)
use byteorder::{LittleEndian, ReadBytesExt};
use serde::Serialize;
use std::io::Cursor;

pub const ARDUPILOT_SERVO_MAGIC: u16 = 18458;
pub const DEFAULT_SENSOR_PORT: u16 = 9003;
pub const DEFAULT_SERVO_PORT: u16 = 9002;
pub const NUM_SERVO_CHANNELS: usize = 16;

#[derive(Debug, Serialize)]
pub struct ImuData {
    pub gyro: [f64; 3],
    pub accel_body: [f64; 3],
}

#[derive(Debug, Serialize)]
pub struct SitlJsonPacket {
    pub timestamp: f64,
    pub imu: ImuData,
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub attitude: [f64; 3],
}

impl SitlJsonPacket {
    pub fn to_json_bytes(&self) -> Vec<u8> {
        let mut buf = serde_json::to_vec(self).expect("JSON serialization cannot fail");
        buf.push(b'\n');
        buf
    }
}

#[derive(Debug, Clone)]
pub struct ServoOutput {
    pub frame_rate: u16,
    pub frame_count: u32,
    pub pwm: [u16; NUM_SERVO_CHANNELS],
}

impl ServoOutput {
    /// Parse ArduPilot's binary servo output packet.
    /// Returns None if the packet is malformed or has wrong magic.
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 4 + 4 + NUM_SERVO_CHANNELS * 2 {
            return None;
        }

        let mut cursor = Cursor::new(data);
        let magic = cursor.read_u16::<LittleEndian>().ok()?;
        if magic != ARDUPILOT_SERVO_MAGIC {
            return None;
        }

        let frame_rate = cursor.read_u16::<LittleEndian>().ok()?;
        let frame_count = cursor.read_u32::<LittleEndian>().ok()?;

        let mut pwm = [0u16; NUM_SERVO_CHANNELS];
        for ch in &mut pwm {
            *ch = cursor.read_u16::<LittleEndian>().ok()?;
        }

        Some(ServoOutput {
            frame_rate,
            frame_count,
            pwm,
        })
    }

    /// Get the first N motor channels as normalized values [0.0, 1.0].
    /// PWM range 1000-2000us maps to 0.0-1.0.
    pub fn motors_normalized(&self, n: usize) -> Vec<f64> {
        self.pwm[..n]
            .iter()
            .map(|&pwm| ((pwm as f64) - 1000.0) / 1000.0)
            .map(|v| v.clamp(0.0, 1.0))
            .collect()
    }
}
