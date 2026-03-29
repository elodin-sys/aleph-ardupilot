/// Elodin-DB integration: subscribe to sensor data, write motor telemetry.
///
/// Sensor data arrives from the STM32 via serial-bridge into Elodin-DB
/// under several vtable namespaces:
///   "aleph"     -- IMU/baro/mag from on-board sensors (BMI270, BMP581, BMM350)
///   "M10Q"      -- GPS from u-blox M10 via STM32 USART2 (J7 connector)
///   "QMC5883L"  -- External compass from QMC5883L via STM32 I2C4 (J7 connector)
///
/// Motor telemetry is written back as a separate vtable for visualization
/// in the Elodin Editor.
use db_macros::{AsVTable, Metadatatize};
use impeller2::types::{LenPacket, PacketId, Timestamp};
use zerocopy::{Immutable, IntoBytes, KnownLayout, TryFromBytes};

/// IMU sensor data from the on-board BMI270/BMM350.
#[derive(AsVTable, Default, Debug, Clone, TryFromBytes, Immutable, KnownLayout)]
#[db(parent = "aleph")]
pub struct SensorInput {
    pub mag: [f32; 3],
    pub gyro: [f32; 3],
    pub accel: [f32; 3],
}

/// GPS data from the Mateksys M10Q-5883 u-blox receiver.
/// Field names map to DB components: M10Q.lat, M10Q.lon, etc.
/// All values use raw UBX-NAV-PVT integer units.
#[derive(AsVTable, Default, Debug, Clone, TryFromBytes, Immutable, KnownLayout)]
#[db(parent = "M10Q")]
pub struct M10QInput {
    pub lat: i32,              // 1e-7 degrees
    pub lon: i32,              // 1e-7 degrees
    pub alt_msl: i32,          // mm above mean sea level
    pub alt_wgs84: i32,        // mm above WGS84 ellipsoid
    pub vel_ned: [i32; 3],     // mm/s [North, East, Down]
    pub fix_type: u8,          // 0=none, 2=2D, 3=3D
    pub satellites: u8,
    pub h_acc: u32,            // horizontal accuracy (mm)
    pub v_acc: u32,            // vertical accuracy (mm)
    pub s_acc: u32,            // speed accuracy (mm/s)
    pub ground_speed: u32,     // mm/s
    pub heading_motion: i32,   // 1e-5 degrees
    pub valid_flags: u8,
    pub ts_us: u64,            // STM32 timestamp (microseconds)
    pub _pad: [u8; 5],
}

/// External compass data from the QMC5883L on the M10Q-5883 module.
/// TODO: simulation -- synthesize QMC5883L data from sim world state for HITL testing.
#[derive(AsVTable, Default, Debug, Clone, TryFromBytes, Immutable, KnownLayout)]
#[db(parent = "QMC5883L")]
pub struct QMC5883LInput {
    pub mag: [i16; 3],         // raw magnetometer LSB
    pub status: u8,
    pub ts_us: u64,            // STM32 timestamp (microseconds)
    pub _pad: u8,
}

/// Motor command telemetry written back to Elodin-DB.
#[derive(AsVTable, Metadatatize, IntoBytes, Immutable, Debug)]
#[db(parent = "ardupilot")]
#[repr(C)]
pub struct MotorTelemetry {
    #[db(timestamp)]
    pub time: i64,
    pub motor_command: [f32; 4],
    pub motor_pwm: [u16; 4],
    pub _pad: [u8; 0],
}

impl MotorTelemetry {
    pub fn new(pwm: [u16; 4], normalized: [f32; 4]) -> Self {
        Self {
            time: Timestamp::now().0,
            motor_command: normalized,
            motor_pwm: pwm,
            _pad: [],
        }
    }

    pub fn to_table_packet(&self, id: PacketId) -> LenPacket {
        let mut table = LenPacket::table(id, core::mem::size_of::<Self>());
        table.extend_from_slice(self.as_bytes());
        table
    }
}
