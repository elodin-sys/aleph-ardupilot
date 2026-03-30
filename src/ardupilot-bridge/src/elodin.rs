/// Elodin-DB integration: subscribe to sensor data, write motor telemetry.
///
/// Sensor data arrives from the STM32 via serial-bridge into Elodin-DB
/// under several vtable namespaces:
///   "IMU"       -- gyro/accel/mag from on-board sensors (BMI270, BMM350) at ~1500 Hz
///   "M10Q"      -- GPS from u-blox M10 via STM32 USART2 (J7 connector)
///   "QMC5883L"  -- External compass from QMC5883L via STM32 I2C4 (J7 connector)
///
/// Motor telemetry is written back as a separate vtable for visualization
/// in the Elodin Editor.
use db_macros::{AsVTable, Metadatatize};
use impeller2::types::{LenPacket, PacketId};
use zerocopy::{Immutable, IntoBytes, KnownLayout, TryFromBytes};

/// IMU sensor data from the on-board BMI270/BMM350 (~1500 Hz).
#[derive(AsVTable, Default, Debug, Clone, TryFromBytes, Immutable, KnownLayout)]
#[db(parent = "IMU")]
pub struct SensorInput {
    pub mag: [f32; 3],
    pub gyro: [f32; 3],
    pub accel: [f32; 3],
}

/// GPS data from the Mateksys M10Q-5883 u-blox receiver.
/// All values use raw UBX-NAV-PVT integer units.
/// repr(C, packed) ensures the struct layout matches the DB wire format
/// exactly -- no alignment padding between fields of different sizes.
#[derive(AsVTable, Default, Debug, Clone, TryFromBytes, Immutable, KnownLayout)]
#[db(parent = "M10Q")]
#[repr(C, packed)]
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
    pub itow: u32,             // GPS time of week (ms)
    pub unix_epoch_ms: i64,    // Unix epoch (ms)
}

/// External compass data from the QMC5883L on the Mateksys M10Q-5883 module.
/// Connected via STM32 I2C4 (J7 connector), mounted away from the Orin NX
/// to minimize magnetic interference.
#[derive(AsVTable, Default, Debug, Clone, TryFromBytes, Immutable, KnownLayout)]
#[db(parent = "QMC5883L")]
#[repr(C, packed)]
pub struct QMC5883LInput {
    pub mag: [i16; 3],         // raw magnetometer LSB (12000 LSB/Gauss at ±2G range)
    pub status: u8,
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
    #[db(skip)]
    pub _pad: [u8; 0],
}

impl MotorTelemetry {
    pub fn new(pwm: [u16; 4], normalized: [f32; 4], time: i64) -> Self {
        Self {
            time,
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
