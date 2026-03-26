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
