/// ArduPilot bridge binary interface.
///
/// Protocol:
///   - Bridge -> ArduPilot: binary sensor packets over UDP
///   - ArduPilot -> Bridge: binary servo/motor output over UDP
///
/// Servo output format:
///   u16 magic (18458)
///   u16 frame_rate
///   u32 frame_count
///   u16[16] pwm_values (microseconds, 1000-2000)
use byteorder::{LittleEndian, ReadBytesExt};
use std::io::Cursor;

pub const ARDUPILOT_SERVO_MAGIC: u16 = 18458;
pub const DEFAULT_SENSOR_PORT: u16 = 9003;
pub const DEFAULT_SERVO_PORT: u16 = 9002;
pub const NUM_SERVO_CHANNELS: usize = 16;
pub const ALEPH_SENSOR_MAGIC: u16 = 0xAE01;
pub const ALEPH_PKT_IMU: u8 = 0x01;
pub const ALEPH_PKT_GPS: u8 = 0x02;
pub const ALEPH_PKT_MAG: u8 = 0x03;
pub const ALEPH_PKT_BARO: u8 = 0x04;

#[derive(Debug, Clone, Copy)]
pub struct AlephImuPacket {
    pub gyro: [f32; 3],
    pub accel_body: [f32; 3],
}

impl AlephImuPacket {
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(2 + 1 + 6 * 4);
        out.extend_from_slice(&ALEPH_SENSOR_MAGIC.to_le_bytes());
        out.push(ALEPH_PKT_IMU);
        for v in self.gyro {
            out.extend_from_slice(&v.to_le_bytes());
        }
        for v in self.accel_body {
            out.extend_from_slice(&v.to_le_bytes());
        }
        out
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AlephGpsPacket {
    pub lat: i32,
    pub lon: i32,
    pub alt_msl: i32,
    pub vel_ned: [i32; 3],
    pub h_acc: u32,
    pub v_acc: u32,
    pub s_acc: u32,
    pub ground_speed: u32,
    pub fix_type: u8,
    pub satellites: u8,
    pub itow: u32,
    pub unix_epoch_ms: i64,
}

impl AlephGpsPacket {
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(2 + 1 + 4 * 10 + 2 + 4 + 8);
        out.extend_from_slice(&ALEPH_SENSOR_MAGIC.to_le_bytes());
        out.push(ALEPH_PKT_GPS);
        out.extend_from_slice(&self.lat.to_le_bytes());
        out.extend_from_slice(&self.lon.to_le_bytes());
        out.extend_from_slice(&self.alt_msl.to_le_bytes());
        for v in self.vel_ned {
            out.extend_from_slice(&v.to_le_bytes());
        }
        out.extend_from_slice(&self.h_acc.to_le_bytes());
        out.extend_from_slice(&self.v_acc.to_le_bytes());
        out.extend_from_slice(&self.s_acc.to_le_bytes());
        out.extend_from_slice(&self.ground_speed.to_le_bytes());
        out.push(self.fix_type);
        out.push(self.satellites);
        out.extend_from_slice(&self.itow.to_le_bytes());
        out.extend_from_slice(&self.unix_epoch_ms.to_le_bytes());
        out
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AlephMagPacket {
    pub mag_mgauss: [f32; 3],
}

impl AlephMagPacket {
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(2 + 1 + 3 * 4);
        out.extend_from_slice(&ALEPH_SENSOR_MAGIC.to_le_bytes());
        out.push(ALEPH_PKT_MAG);
        for v in self.mag_mgauss {
            out.extend_from_slice(&v.to_le_bytes());
        }
        out
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AlephBaroPacket {
    pub pressure_pa: f32,
    pub temperature_c: f32,
}

impl AlephBaroPacket {
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(2 + 1 + 2 * 4);
        out.extend_from_slice(&ALEPH_SENSOR_MAGIC.to_le_bytes());
        out.push(ALEPH_PKT_BARO);
        out.extend_from_slice(&self.pressure_pa.to_le_bytes());
        out.extend_from_slice(&self.temperature_c.to_le_bytes());
        out
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
