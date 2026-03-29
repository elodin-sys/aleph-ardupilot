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

/// Convert quaternion from Elodin convention (scalar-last [x,y,z,w])
/// to ArduPilot convention (scalar-first [w,x,y,z]), with ENU->NED rotation.
///
/// The ENU->NED rotation is a 180-degree rotation about the body X axis
/// followed by a 90-degree rotation about the Z axis. In practice, for
/// quaternion attitude representation:
///   q_ned = R_enu_to_ned * q_enu
///
/// For the SITL JSON interface, ArduPilot accepts roll/pitch/yaw in radians
/// rather than quaternions, so this may not be needed initially.
pub fn quat_enu_to_ned_wxyz(xyzw: [f64; 4]) -> [f64; 4] {
    // Reorder scalar-last to scalar-first, then apply frame transform.
    // ENU->NED quaternion mapping: swap x<->y, negate z, keep w.
    let [x, y, z, w] = xyzw;
    [w, y, x, -z]
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
