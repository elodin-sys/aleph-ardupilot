/// Elodin-DB integration: subscribe to sensor data, write motor telemetry.
///
/// Sensor data arrives from the STM32 via serial-bridge into Elodin-DB
/// as the "aleph" vtable with fields: mag, gyro, accel, baro, etc.
/// We subscribe to this table and forward to ArduPilot.
///
/// Motor telemetry is written back as a separate vtable for visualization
/// in the Elodin Editor.
use db_macros::{AsVTable, Metadatatize};
use impeller2::types::{LenPacket, PacketId, Timestamp};
use zerocopy::{Immutable, IntoBytes, KnownLayout, TryFromBytes};

/// Sensor data schema matching the upstream serial-bridge output.
/// The field layout must match the `BridgeRecord` struct in serial-bridge.
#[derive(AsVTable, Default, Debug, Clone, TryFromBytes, Immutable, KnownLayout)]
#[db(parent = "aleph")]
pub struct SensorInput {
    pub mag: [f32; 3],
    pub gyro: [f32; 3],
    pub accel: [f32; 3],
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
