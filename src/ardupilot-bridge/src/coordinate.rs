/// Coordinate and unit conversions for the ardupilot-bridge.
///
/// Sensor data from the STM32 (via sensor-fw) arrives in:
///   - Body frame: FRD (Forward-Right-Down)
///   - Accel units: g (1.0 = 9.80665 m/s²)
///   - Gyro units: degrees/second
///
/// ArduPilot SITL JSON expects:
///   - Body frame: FRD (same)
///   - Accel units: m/s²
///   - Gyro units: rad/s
///
/// The MEKF outputs q_hat as a scalar-last [x,y,z,w] quaternion in
/// FRD body / NED-like world frame (gravity ref = [0,0,1]).

pub const GRAVITY_MSS: f64 = 9.80665;
const DEG_TO_RAD: f64 = std::f64::consts::PI / 180.0;

/// Convert accelerometer reading from g to m/s².
/// Sensor-fw outputs FRD with gravity as [0, 0, +1.0] g for a level drone.
/// ArduPilot expects FRD specific force: [0, 0, -GRAVITY] m/s² for level.
/// We negate to convert from "gravity direction" to "specific force" convention.
pub fn accel_g_to_ms2(g: [f64; 3]) -> [f64; 3] {
    [
        -g[0] * GRAVITY_MSS,
        -g[1] * GRAVITY_MSS,
        -g[2] * GRAVITY_MSS,
    ]
}

/// Convert gyroscope reading from degrees/second to radians/second.
/// No frame conversion needed -- sensor-fw already outputs FRD.
pub fn gyro_dps_to_rads(dps: [f64; 3]) -> [f64; 3] {
    [
        dps[0] * DEG_TO_RAD,
        dps[1] * DEG_TO_RAD,
        dps[2] * DEG_TO_RAD,
    ]
}

// ---------------------------------------------------------------------------
// MEKF quaternion -> Euler (NED/FRD)
// ---------------------------------------------------------------------------

/// Convert MEKF quaternion to ArduPilot [roll, pitch, yaw] in NED/FRD radians.
///
/// The MEKF already operates in FRD body frame with gravity reference [0,0,1]
/// (NED-like world frame), so no frame rotation is needed -- just extract
/// Euler angles directly from the scalar-last [x,y,z,w] quaternion.
pub fn mekf_quat_to_euler(q_xyzw: [f64; 4]) -> [f64; 3] {
    let x = q_xyzw[0];
    let y = q_xyzw[1];
    let z = q_xyzw[2];
    let w = q_xyzw[3];

    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = sinr_cosp.atan2(cosr_cosp);

    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        sinp.signum() * std::f64::consts::FRAC_PI_2
    } else {
        sinp.asin()
    };

    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = siny_cosp.atan2(cosy_cosp);

    [roll, pitch, yaw]
}

// ---------------------------------------------------------------------------
// QMC5883L magnetometer conversions
// ---------------------------------------------------------------------------

const QMC5883L_LSB_PER_GAUSS: f64 = 3000.0; // ±8G range sensitivity

/// Convert raw QMC5883L i16 LSB readings to Gauss.
pub fn qmc_raw_to_gauss(raw: [i16; 3]) -> [f64; 3] {
    [
        raw[0] as f64 / QMC5883L_LSB_PER_GAUSS,
        raw[1] as f64 / QMC5883L_LSB_PER_GAUSS,
        raw[2] as f64 / QMC5883L_LSB_PER_GAUSS,
    ]
}

// ---------------------------------------------------------------------------
// Tilt-compensated compass attitude
// ---------------------------------------------------------------------------

/// Compute [roll, pitch, yaw] in NED/FRD radians from accelerometer and
/// magnetometer readings in body frame.
///
/// The accelerometer input is *specific force* in FRD m/s² (a level drone
/// reads approximately [0, 0, -9.81]).  The formula internally negates it
/// to get the gravity direction vector (positive Z = down in FRD) before
/// computing roll and pitch.
///
/// The magnetometer input is in the QMC5883L sensor frame, which is treated
/// as aligned with FRD for now. The yaw is derived after tilt compensation.
pub fn tilt_compensated_attitude(accel_frd: [f64; 3], mag: [f64; 3]) -> [f64; 3] {
    // Negate sensed force to get gravity direction (down = positive Z in FRD).
    let gx = -accel_frd[0];
    let gy = -accel_frd[1];
    let gz = -accel_frd[2];

    let roll = gy.atan2(gz);
    let sr = roll.sin();
    let cr = roll.cos();

    let pitch = (-gx).atan2(gy * sr + gz * cr);
    let sp = pitch.sin();
    let cp = pitch.cos();

    let mx = mag[0] * cp
           + mag[1] * sr * sp
           + mag[2] * cr * sp;
    let my = mag[1] * cr
           - mag[2] * sr;
    let yaw = (-my).atan2(mx);

    [roll, pitch, yaw]
}

// ---------------------------------------------------------------------------
// GPS coordinate conversions (raw UBX integer units -> ArduPilot NED)
// ---------------------------------------------------------------------------

const WGS84_A: f64 = 6378137.0; // semi-major axis (meters)

/// Convert UBX lat/lon (1e-7 degrees, i32) + altitude (mm, i32) to NED
/// meters relative to the ArduPilot home location.
pub fn ubx_lla_to_ned(
    lat_e7: i32,
    lon_e7: i32,
    alt_mm: i32,
    home_lat_deg: f64,
    home_lon_deg: f64,
    home_alt_m: f64,
) -> [f64; 3] {
    let lat_deg = lat_e7 as f64 * 1e-7;
    let lon_deg = lon_e7 as f64 * 1e-7;
    let alt_m = alt_mm as f64 * 1e-3;

    let dlat_rad = (lat_deg - home_lat_deg).to_radians();
    let dlon_rad = (lon_deg - home_lon_deg).to_radians();

    let north = dlat_rad * WGS84_A;
    let east = dlon_rad * WGS84_A * home_lat_deg.to_radians().cos();
    let down = -(alt_m - home_alt_m);

    [north, east, down]
}

/// Convert UBX vel_ned [i32; 3] from mm/s to m/s as [f64; 3].
pub fn ubx_vel_to_ms(vel_ned_mm: [i32; 3]) -> [f64; 3] {
    [
        vel_ned_mm[0] as f64 * 1e-3,
        vel_ned_mm[1] as f64 * 1e-3,
        vel_ned_mm[2] as f64 * 1e-3,
    ]
}

/// Convert UBX heading_motion (1e-5 degrees, i32) to radians.
pub fn ubx_heading_to_rad(heading_e5: i32) -> f64 {
    (heading_e5 as f64 * 1e-5).to_radians()
}

// ---------------------------------------------------------------------------
// MEKF-synthesized compass (Earth field rotated from NED to body)
// ---------------------------------------------------------------------------

/// Synthesize body-frame magnetic field (milliGauss) from MEKF attitude.
///
/// Rotates the expected Earth field from NED to body frame using the
/// standard aerospace ZYX rotation (yaw, pitch, roll). This produces
/// compass readings that match ArduPilot's World Magnetic Model
/// expectations, giving EKF3 a clean heading source from the MEKF.
pub fn synthesize_mag_field_mgauss(
    roll: f64,
    pitch: f64,
    yaw: f64,
    earth_b_horiz_mg: f64,
    earth_declination_rad: f64,
    earth_b_down_mg: f64,
) -> [f32; 3] {
    let bn = earth_b_horiz_mg * earth_declination_rad.cos();
    let be = earth_b_horiz_mg * earth_declination_rad.sin();
    let bd = earth_b_down_mg;

    let sr = roll.sin();
    let cr = roll.cos();
    let sp = pitch.sin();
    let cp = pitch.cos();
    let sy = yaw.sin();
    let cy = yaw.cos();

    // NED-to-body rotation matrix R = Rx(roll) * Ry(pitch) * Rz(yaw)
    let bx = bn * (cp * cy) + be * (cp * sy) + bd * (-sp);
    let by = bn * (sr * sp * cy - cr * sy) + be * (sr * sp * sy + cr * cy) + bd * (sr * cp);
    let bz = bn * (cr * sp * cy + sr * sy) + be * (cr * sp * sy - sr * cy) + bd * (cr * cp);

    [bx as f32, by as f32, bz as f32]
}

// ---------------------------------------------------------------------------
// Heading fusion
// ---------------------------------------------------------------------------

/// Blend two angles (radians) with correct wraparound handling.
/// `alpha` is the blend factor: 0.0 = purely `a`, 1.0 = purely `b`.
pub fn blend_angle(a: f64, b: f64, alpha: f64) -> f64 {
    let mut diff = b - a;
    // Normalize diff to [-pi, pi]
    while diff > std::f64::consts::PI {
        diff -= 2.0 * std::f64::consts::PI;
    }
    while diff < -std::f64::consts::PI {
        diff += 2.0 * std::f64::consts::PI;
    }
    let mut result = a + alpha * diff;
    // Normalize result to [-pi, pi]
    while result > std::f64::consts::PI {
        result -= 2.0 * std::f64::consts::PI;
    }
    while result < -std::f64::consts::PI {
        result += 2.0 * std::f64::consts::PI;
    }
    result
}
