/// Coordinate frame conversions between Elodin (ENU/FLU) and ArduPilot (NED/FRD).
///
/// Elodin uses:
///   - World frame: ENU (East-North-Up)
///   - Body frame: FLU (Forward-Left-Up)
///
/// ArduPilot SITL expects:
///   - World frame: NED (North-East-Down)
///   - Body frame: FRD (Forward-Right-Down)

/// Convert a 3-vector from ENU world frame to NED world frame.
/// ENU [e, n, u] -> NED [n, e, -u]
pub fn enu_to_ned(enu: [f64; 3]) -> [f64; 3] {
    [enu[1], enu[0], -enu[2]]
}

/// Convert a 3-vector from FLU body frame to FRD body frame.
/// FLU [f, l, u] -> FRD [f, -l, -u]
pub fn flu_to_frd(flu: [f64; 3]) -> [f64; 3] {
    [flu[0], -flu[1], -flu[2]]
}

/// Convert angular velocity from FLU to FRD.
/// Same transformation as flu_to_frd: negate Y and Z.
pub fn gyro_flu_to_frd(flu: [f64; 3]) -> [f64; 3] {
    flu_to_frd(flu)
}

/// Convert acceleration from FLU to FRD.
/// Same transformation as flu_to_frd: negate Y and Z.
pub fn accel_flu_to_frd(flu: [f64; 3]) -> [f64; 3] {
    flu_to_frd(flu)
}

// ---------------------------------------------------------------------------
// MEKF quaternion conversions (Elodin ENU/FLU -> ArduPilot NED/FRD)
// ---------------------------------------------------------------------------

fn quat_xyzw_to_rotmat(q: [f64; 4]) -> [[f64; 3]; 3] {
    let x = q[0];
    let y = q[1];
    let z = q[2];
    let w = q[3];

    let xx = x * x;
    let yy = y * y;
    let zz = z * z;
    let xy = x * y;
    let xz = x * z;
    let yz = y * z;
    let wx = w * x;
    let wy = w * y;
    let wz = w * z;

    [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]
}

fn rotmat_to_quat_wxyz(r: [[f64; 3]; 3]) -> [f64; 4] {
    let trace = r[0][0] + r[1][1] + r[2][2];
    let (w, x, y, z) = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        (
            0.25 * s,
            (r[2][1] - r[1][2]) / s,
            (r[0][2] - r[2][0]) / s,
            (r[1][0] - r[0][1]) / s,
        )
    } else if r[0][0] > r[1][1] && r[0][0] > r[2][2] {
        let s = (1.0 + r[0][0] - r[1][1] - r[2][2]).sqrt() * 2.0;
        (
            (r[2][1] - r[1][2]) / s,
            0.25 * s,
            (r[0][1] + r[1][0]) / s,
            (r[0][2] + r[2][0]) / s,
        )
    } else if r[1][1] > r[2][2] {
        let s = (1.0 + r[1][1] - r[0][0] - r[2][2]).sqrt() * 2.0;
        (
            (r[0][2] - r[2][0]) / s,
            (r[0][1] + r[1][0]) / s,
            0.25 * s,
            (r[1][2] + r[2][1]) / s,
        )
    } else {
        let s = (1.0 + r[2][2] - r[0][0] - r[1][1]).sqrt() * 2.0;
        (
            (r[1][0] - r[0][1]) / s,
            (r[0][2] + r[2][0]) / s,
            (r[1][2] + r[2][1]) / s,
            0.25 * s,
        )
    };

    [w, x, y, z]
}

fn mat_mul_3x3(a: [[f64; 3]; 3], b: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut out = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
    out
}

/// Convert a quaternion from Elodin ENU/FLU to ArduPilot NED/FRD as [w, x, y, z].
///
/// Input `q_enu_xyzw` is expected in [x, y, z, w] order from `aleph.q_hat`.
pub fn quat_enu_to_ned_wxyz(q_enu_xyzw: [f64; 4]) -> [f64; 4] {
    let r_enu_flu = quat_xyzw_to_rotmat(q_enu_xyzw);
    let c_w = [
        [0.0, 1.0, 0.0], // ENU -> NED (world)
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ];
    let c_b = [
        [1.0, 0.0, 0.0], // FRD -> FLU (body)
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
    ];

    let r_ned_frd = mat_mul_3x3(mat_mul_3x3(c_w, r_enu_flu), c_b);
    rotmat_to_quat_wxyz(r_ned_frd)
}

/// Convert MEKF quaternion (`aleph.q_hat`) to ArduPilot [roll, pitch, yaw] in
/// NED/FRD radians.
pub fn mekf_quat_to_euler_ned(q_hat_xyzw: [f64; 4]) -> [f64; 3] {
    let q = quat_enu_to_ned_wxyz(q_hat_xyzw);
    let w = q[0];
    let x = q[1];
    let y = q[2];
    let z = q[3];

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
/// The accelerometer input is *sensed force* in FRD (a level drone reads
/// approximately [0, 0, -9.81]).  The formula internally negates it to get
/// the gravity direction vector (positive Z = down in FRD) before computing
/// roll and pitch.
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
