mod ardupilot_ipc;
mod can_output;
mod config;
mod coordinate;
mod elodin;
mod hitl;

use crate::ardupilot_ipc::{ServoOutput, SitlJsonPacket, ImuData};
use crate::can_output::CanOutput;
use crate::config::{Config, HomeLocation};
use crate::coordinate::{
    gyro_flu_to_frd, accel_flu_to_frd,
    ubx_lla_to_ned, ubx_vel_to_ms, ubx_heading_to_rad,
    qmc_raw_to_gauss, mekf_quat_to_euler_ned, blend_angle,
};
use crate::elodin::{M10QInput, QMC5883LInput, MekfInput, MotorTelemetry, SensorInput};

use anyhow::Context;
use clap::Parser;
use impeller2::types::{PacketId, Timestamp};
use impeller2_stellar::{Client, SinkExt, StreamExt};
use std::net::{SocketAddr, UdpSocket};
use std::sync::{Arc, Mutex};
use std::time::Duration;

fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt::init();
    stellarator::run(run)
}

/// Cached GPS state, updated by the GPS subscription task at ~2.5 Hz.
#[derive(Debug, Clone, Copy)]
struct GpsCache {
    position_ned: [f64; 3],
    velocity_ned: [f64; 3],
    yaw_rad: f64,
    has_fix: bool,
    satellites: u8,
    h_acc_m: f64,
    ground_speed_ms: f64,
    unix_epoch_ms: i64,
    epoch_us: i64,
    local_us: i64,
}

impl Default for GpsCache {
    fn default() -> Self {
        Self {
            position_ned: [0.0; 3],
            velocity_ned: [0.0; 3],
            yaw_rad: 0.0,
            has_fix: false,
            satellites: 0,
            h_acc_m: 999.0,
            ground_speed_ms: 0.0,
            unix_epoch_ms: 0,
            epoch_us: 0,
            local_us: 0,
        }
    }
}

/// Cached IMU sample from the STM32 pre-integrated stream (~750 Hz).
/// Written by the IMU background thread, read by the main loop for
/// JSON packet construction.
#[derive(Debug, Clone, Copy)]
struct ImuCache {
    gyro: [f64; 3],      // FRD rad/s
    accel: [f64; 3],     // FRD m/s^2
    timestamp: f64,       // seconds since bridge start
    seq: u64,             // monotonic sample count
}

impl Default for ImuCache {
    fn default() -> Self {
        Self {
            gyro: [0.0; 3],
            accel: [0.0; 3],
            timestamp: 0.0,
            seq: 0,
        }
    }
}

/// Cached MEKF attitude, updated from aleph.q_hat at high rate.
#[derive(Debug, Clone, Copy)]
struct AttitudeCache {
    roll: f64,
    pitch: f64,
    yaw: f64,
    valid: bool,
}

impl Default for AttitudeCache {
    fn default() -> Self {
        Self {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            valid: false,
        }
    }
}

/// Cached QMC5883L magnetometer reading, updated by the mag subscription
/// task at ~100-200 Hz. Used with the IMU accelerometer to compute a
/// tilt-compensated compass attitude for ArduPilot SITL.
#[derive(Debug, Clone, Copy)]
struct MagCache {
    mag_frd: [f64; 3],   // body-frame FRD magnetic field in Gauss
    valid: bool,
}

impl Default for MagCache {
    fn default() -> Self {
        Self {
            mag_frd: [0.0; 3],
            valid: false,
        }
    }
}

async fn run() -> anyhow::Result<()> {
    let config = Config::parse();
    tracing::info!("ardupilot-bridge starting with config: {:?}", config);

    let home = HomeLocation::parse(&config.home)
        .context("parse --home / AP_HOME")?;
    tracing::info!(
        "home location: lat={:.6} lon={:.6} alt={:.1}m",
        home.lat_deg, home.lon_deg, home.alt_m,
    );

    let telemetry_id: PacketId = fastrand::u16(..).to_le_bytes();

    // Bind UDP once -- reused across Elodin-DB reconnects.
    let bind_addr: SocketAddr = format!("0.0.0.0:{}", config.servo_port)
        .parse()
        .unwrap();
    let udp_socket = UdpSocket::bind(bind_addr)
        .context("bind UDP control port")?;
    udp_socket.set_nonblocking(true)
        .context("set UDP non-blocking")?;

    let mut can = CanOutput::new(&config.can_interface);
    can.open().context("open CAN interface")?;

    tracing::info!(
        "bridge bound to UDP :{}, waiting for ArduPilot servo output, CAN={}",
        config.servo_port,
        if can.is_enabled() { &config.can_interface } else { "disabled" }
    );

    if config.hitl_port > 0 {
        let hitl_port = config.hitl_port;
        let hitl_udp = udp_socket.try_clone().context("clone UDP for HITL")?;
        std::thread::spawn(move || {
            let mut hitl_ap_addr: Option<SocketAddr> = None;
            if let Err(e) = hitl::run_hitl_loop(hitl_port, &hitl_udp, &mut hitl_ap_addr) {
                tracing::error!("HITL server error: {}", e);
            }
        });
    }

    // GPS subscription -- background thread, ~2.5 Hz from STM32 UBX parser.
    let gps_cache = Arc::new(Mutex::new(GpsCache::default()));
    {
        let gps_cache = Arc::clone(&gps_cache);
        let home = home;
        let elodin_addr_str = config.elodin_addr.clone();
        std::thread::spawn(move || {
            if let Err(e) = stellarator::run(move || gps_task(elodin_addr_str, home, gps_cache)) {
                tracing::error!("GPS task fatal: {:?}", e);
            }
        });
    }

    // QMC5883L magnetometer subscription -- background thread, ~100-200 Hz.
    let mag_cache = Arc::new(Mutex::new(MagCache::default()));
    {
        let mag_cache = Arc::clone(&mag_cache);
        let elodin_addr_str = config.elodin_addr.clone();
        std::thread::spawn(move || {
            if let Err(e) = stellarator::run(move || qmc5883l_task(elodin_addr_str, mag_cache)) {
                tracing::error!("QMC5883L task fatal: {:?}", e);
            }
        });
    }

    // MEKF attitude subscription -- background thread, high-rate aleph.q_hat.
    let attitude_cache = Arc::new(Mutex::new(AttitudeCache::default()));
    {
        let attitude_cache = Arc::clone(&attitude_cache);
        let elodin_addr_str = config.elodin_addr.clone();
        std::thread::spawn(move || {
            if let Err(e) = stellarator::run(move || mekf_task(elodin_addr_str, attitude_cache)) {
                tracing::error!("MEKF task fatal: {:?}", e);
            }
        });
    }

    // IMU subscription -- background thread, ~1500 Hz from STM32 sensor-fw.
    let imu_cache = Arc::new(Mutex::new(ImuCache::default()));
    {
        let imu_cache = Arc::clone(&imu_cache);
        let elodin_addr_str = config.elodin_addr.clone();
        std::thread::spawn(move || {
            if let Err(e) = stellarator::run(move || imu_task(elodin_addr_str, imu_cache)) {
                tracing::error!("IMU task fatal: {:?}", e);
            }
        });
    }

    // Retry loop -- only the Elodin-DB connection is re-established on error.
    // The UDP socket and background threads survive across retries.
    loop {
        if let Err(err) = bridge_loop(
            &config, telemetry_id,
            &udp_socket, &mut can,
            &gps_cache, &mag_cache, &imu_cache, &attitude_cache,
        ).await {
            tracing::error!("bridge error: {:?}", err);
            stellarator::sleep(Duration::from_millis(500)).await;
        }
    }
}

async fn bridge_loop(
    config: &Config,
    telemetry_id: PacketId,
    udp_socket: &UdpSocket,
    can: &mut CanOutput,
    gps_cache: &Arc<Mutex<GpsCache>>,
    _mag_cache: &Arc<Mutex<MagCache>>,
    imu_cache: &Arc<Mutex<ImuCache>>,
    attitude_cache: &Arc<Mutex<AttitudeCache>>,
) -> anyhow::Result<()> {
    let elodin_addr: SocketAddr = config
        .elodin_addr
        .parse()
        .context("invalid elodin_addr")?;

    tracing::info!("connecting to Elodin-DB at {}", elodin_addr);

    let mut client = Client::connect(elodin_addr)
        .await
        .map_err(anyhow::Error::from)?;
    client.init_world::<MotorTelemetry>(telemetry_id).await?;

    tracing::info!("connected to Elodin-DB, entering main loop");

    let mut ap_addr: Option<SocketAddr> = None;
    let mut last_imu_seq: u64 = 0;

    let mut rate_timer = std::time::Instant::now();
    let mut json_sent: u64 = 0;
    let mut servo_recv: u64 = 0;
    let mut imu_skipped: u64 = 0;
    let mut loop_iters: u64 = 0;

    loop {
        loop_iters += 1;

        let mut servo_buf = [0u8; 256];
        if let Ok((n, src)) = udp_socket.recv_from(&mut servo_buf) {
            if let Some(servo) = ServoOutput::from_bytes(&servo_buf[..n]) {
                if ap_addr.is_none() {
                    tracing::info!("ArduPilot connected from {}", src);
                }
                ap_addr = Some(src);
                servo_recv += 1;

                let motors_norm = servo.motors_normalized(config.num_motors);
                let motor_pwm: [u16; 4] = [
                    servo.pwm[0],
                    servo.pwm[1],
                    servo.pwm[2],
                    servo.pwm[3],
                ];
                let motor_cmd: [f32; 4] = [
                    motors_norm.get(0).copied().unwrap_or(0.0) as f32,
                    motors_norm.get(1).copied().unwrap_or(0.0) as f32,
                    motors_norm.get(2).copied().unwrap_or(0.0) as f32,
                    motors_norm.get(3).copied().unwrap_or(0.0) as f32,
                ];

                if can.is_enabled() {
                    if let Err(e) = can.send_esc_command(&motors_norm) {
                        tracing::warn!("CAN send error: {}", e);
                    }
                }

                let local_now_us = Timestamp::now().0;
                let telemetry_time_us = {
                    let gps = gps_cache.lock().unwrap();
                    if gps.epoch_us > 0 {
                        gps.epoch_us
                            .saturating_add(local_now_us.saturating_sub(gps.local_us))
                    } else {
                        local_now_us
                    }
                };

                let telemetry = MotorTelemetry::new(motor_pwm, motor_cmd, telemetry_time_us);
                let table = telemetry.to_table_packet(telemetry_id);
                client.send(table).await.0?;
            }
        }

        if ap_addr.is_none() {
            continue;
        }
        let target = ap_addr.unwrap();

        let imu = {
            let c = imu_cache.lock().unwrap();
            *c
        };

        if imu.seq == 0 || imu.seq == last_imu_seq {
            stellarator::sleep(Duration::from_micros(500)).await;
            continue;
        }

        let skipped = imu.seq.saturating_sub(last_imu_seq).saturating_sub(1);
        imu_skipped += skipped;
        last_imu_seq = imu.seq;
        json_sent += 1;

        let gps = gps_cache.lock().unwrap();
        let (position, velocity) = if gps.has_fix {
            (gps.position_ned, gps.velocity_ned)
        } else {
            ([0.0; 3], [0.0; 3])
        };
        let gps_yaw = gps.yaw_rad;
        let gps_fix = gps.has_fix;
        let ground_speed_ms = gps.ground_speed_ms;
        drop(gps);

        let mekf = {
            let c = attitude_cache.lock().unwrap();
            *c
        };

        let gx = -imu.accel[0];
        let gy = -imu.accel[1];
        let gz = -imu.accel[2];
        let roll = gy.atan2(gz);
        let pitch = (-gx).atan2(gy * roll.sin() + gz * roll.cos());
        let yaw = if mekf.valid {
            if !gps_fix {
                mekf.yaw
            } else if ground_speed_ms > 3.0 {
                gps_yaw
            } else if ground_speed_ms > 1.0 {
                let alpha = (ground_speed_ms - 1.0) / 2.0;
                blend_angle(mekf.yaw, gps_yaw, alpha)
            } else {
                mekf.yaw
            }
        } else if gps_fix {
            gps_yaw
        } else {
            0.0
        };
        let attitude = [roll, pitch, yaw];

        let packet = SitlJsonPacket {
            timestamp: imu.timestamp,
            imu: ImuData {
                gyro: imu.gyro,
                accel_body: imu.accel,
            },
            position,
            velocity,
            attitude,
        };

        let json_bytes = packet.to_json_bytes();
        udp_socket
            .send_to(&json_bytes, target)
            .context("send sensor JSON to ArduPilot")?;

        let elapsed = rate_timer.elapsed();
        if elapsed >= Duration::from_secs(1) {
            let secs = elapsed.as_secs_f64();
            let imu_db_hz = {
                let c = imu_cache.lock().unwrap();
                c.seq as f64 / c.timestamp.max(0.001)
            };
            tracing::info!(
                "RATES: imu_from_db={:.0}Hz json_to_ap={:.0}Hz servo_from_ap={:.0}Hz imu_skipped={} loop={:.0}/s",
                imu_db_hz,
                json_sent as f64 / secs,
                servo_recv as f64 / secs,
                imu_skipped,
                loop_iters as f64 / secs,
            );
            rate_timer = std::time::Instant::now();
            json_sent = 0;
            servo_recv = 0;
            imu_skipped = 0;
            loop_iters = 0;
        }
    }
}

// ---------------------------------------------------------------------------
// Background tasks: IMU, GPS, and QMC5883L subscriptions
// ---------------------------------------------------------------------------

/// IMU subscription task. Runs in a dedicated thread with its own stellarator
/// runtime and Elodin-DB connection. Forwards pre-integrated samples from the
/// STM32 (~750 Hz) to the ImuCache after FLU-to-FRD conversion.
async fn imu_task(
    elodin_addr: String,
    cache: Arc<Mutex<ImuCache>>,
) -> anyhow::Result<()> {
    let start = std::time::Instant::now();
    loop {
        match imu_subscribe_loop(&elodin_addr, &cache, &start).await {
            Ok(()) => {}
            Err(e) => {
                tracing::warn!("IMU subscription error (will retry): {}", e);
                stellarator::sleep(Duration::from_secs(1)).await;
            }
        }
    }
}

async fn imu_subscribe_loop(
    elodin_addr: &str,
    cache: &Arc<Mutex<ImuCache>>,
    start: &std::time::Instant,
) -> anyhow::Result<()> {
    let addr: SocketAddr = elodin_addr.parse().context("parse elodin addr for IMU")?;
    let mut client = Client::connect(addr).await.map_err(anyhow::Error::from)?;
    let mut sub = client.subscribe::<SensorInput>().await?;
    tracing::info!("IMU: subscribed to IMU vtable");

    let mut seq: u64 = 0;

    loop {
        let input = sub.next().await?;
        seq += 1;
        let timestamp = start.elapsed().as_secs_f64();

        let gyro_frd = gyro_flu_to_frd([
            input.gyro[0] as f64,
            input.gyro[1] as f64,
            input.gyro[2] as f64,
        ]);
        let accel_frd = accel_flu_to_frd([
            input.accel[0] as f64,
            input.accel[1] as f64,
            input.accel[2] as f64,
        ]);

        {
            let mut c = cache.lock().unwrap();
            c.gyro = gyro_frd;
            c.accel = accel_frd;
            c.timestamp = timestamp;
            c.seq = seq;
        }

        if seq % 10000 == 1 {
            let hz = seq as f64 / timestamp.max(0.001);
            tracing::info!("IMU: samples={} rate={:.0} Hz", seq, hz);
        }
    }
}

/// GPS subscription task. Reconnects on error.
/// TODO: simulation -- synthetic M10Q rows from sim for HITL testing.
async fn gps_task(
    elodin_addr: String,
    home: HomeLocation,
    cache: Arc<Mutex<GpsCache>>,
) -> anyhow::Result<()> {
    loop {
        match gps_subscribe_loop(&elodin_addr, &home, &cache).await {
            Ok(()) => {}
            Err(e) => {
                tracing::warn!("GPS subscription error (will retry): {}", e);
                stellarator::sleep(Duration::from_secs(2)).await;
            }
        }
    }
}

async fn gps_subscribe_loop(
    elodin_addr: &str,
    home: &HomeLocation,
    cache: &Arc<Mutex<GpsCache>>,
) -> anyhow::Result<()> {
    let addr: SocketAddr = elodin_addr.parse().context("parse elodin addr for GPS")?;
    let mut client = Client::connect(addr).await.map_err(anyhow::Error::from)?;
    let mut sub = client.subscribe::<M10QInput>().await?;
    tracing::info!("GPS: subscribed to M10Q vtable");

    let mut gps_tick: u64 = 0;
    loop {
        let gps = sub.next().await?;
        let has_fix = gps.fix_type >= 3;

        let position_ned = if has_fix {
            ubx_lla_to_ned(
                gps.lat, gps.lon, gps.alt_msl,
                home.lat_deg, home.lon_deg, home.alt_m,
            )
        } else {
            [0.0; 3]
        };

        let velocity_ned = if has_fix {
            ubx_vel_to_ms(gps.vel_ned)
        } else {
            [0.0; 3]
        };

        let yaw_rad = ubx_heading_to_rad(gps.heading_motion);
        let h_acc_m = gps.h_acc as f64 * 1e-3;
        let ground_speed_ms = gps.ground_speed as f64 * 1e-3;

        {
            let mut c = cache.lock().unwrap();
            c.position_ned = position_ned;
            c.velocity_ned = velocity_ned;
            c.yaw_rad = yaw_rad;
            c.has_fix = has_fix;
            c.satellites = gps.satellites;
            c.h_acc_m = h_acc_m;
            c.ground_speed_ms = ground_speed_ms;
            c.unix_epoch_ms = gps.unix_epoch_ms;
            if gps.unix_epoch_ms > 0 {
                c.epoch_us = gps.unix_epoch_ms.saturating_mul(1000);
                c.local_us = Timestamp::now().0;
            }
        }

        gps_tick += 1;
        if gps_tick % 10 == 1 {
            tracing::info!(
                "GPS: fix={} sats={} lat={:.7} lon={:.7} alt={:.1}m h_acc={:.2}m NED=[{:.1},{:.1},{:.1}]",
                gps.fix_type,
                gps.satellites,
                gps.lat as f64 * 1e-7,
                gps.lon as f64 * 1e-7,
                gps.alt_msl as f64 * 1e-3,
                h_acc_m,
                position_ned[0], position_ned[1], position_ned[2],
            );
        }
    }
}

/// QMC5883L magnetometer subscription task. Reconnects on error.
/// Subscribes to QMC5883L.mag (raw i16 LSB) and converts to body-frame
/// Gauss values for tilt-compensated compass computation.
async fn qmc5883l_task(
    elodin_addr: String,
    cache: Arc<Mutex<MagCache>>,
) -> anyhow::Result<()> {
    loop {
        match qmc5883l_subscribe_loop(&elodin_addr, &cache).await {
            Ok(()) => {}
            Err(e) => {
                tracing::warn!("QMC5883L subscription error (will retry): {}", e);
                stellarator::sleep(Duration::from_secs(2)).await;
            }
        }
    }
}

async fn qmc5883l_subscribe_loop(
    elodin_addr: &str,
    cache: &Arc<Mutex<MagCache>>,
) -> anyhow::Result<()> {
    let addr: SocketAddr = elodin_addr.parse().context("parse elodin addr for QMC5883L")?;
    let mut client = Client::connect(addr).await.map_err(anyhow::Error::from)?;
    let mut sub = client.subscribe::<QMC5883LInput>().await?;
    tracing::info!("QMC5883L: subscribed to QMC5883L vtable");

    let mut mag_tick: u64 = 0;
    loop {
        let input = sub.next().await?;
        let mag_gauss = qmc_raw_to_gauss(input.mag);

        {
            let mut c = cache.lock().unwrap();
            c.mag_frd = mag_gauss;
            c.valid = true;
        }

        mag_tick += 1;
        if mag_tick % 200 == 1 {
            let magnitude = (mag_gauss[0].powi(2) + mag_gauss[1].powi(2) + mag_gauss[2].powi(2)).sqrt();
            tracing::info!(
                "MAG: raw=[{:.4},{:.4},{:.4}] |B|={:.4} G",
                mag_gauss[0], mag_gauss[1], mag_gauss[2], magnitude,
            );
        }
    }
}

/// MEKF attitude subscription task. Reconnects on error.
/// Subscribes to `aleph.q_hat`, converts ENU/FLU quaternion to NED/FRD Euler.
async fn mekf_task(
    elodin_addr: String,
    cache: Arc<Mutex<AttitudeCache>>,
) -> anyhow::Result<()> {
    loop {
        match mekf_subscribe_loop(&elodin_addr, &cache).await {
            Ok(()) => {}
            Err(e) => {
                tracing::warn!("MEKF subscription error (will retry): {}", e);
                stellarator::sleep(Duration::from_secs(2)).await;
            }
        }
    }
}

async fn mekf_subscribe_loop(
    elodin_addr: &str,
    cache: &Arc<Mutex<AttitudeCache>>,
) -> anyhow::Result<()> {
    let addr: SocketAddr = elodin_addr.parse().context("parse elodin addr for MEKF")?;
    let mut client = Client::connect(addr).await.map_err(anyhow::Error::from)?;
    let mut sub = client.subscribe::<MekfInput>().await?;
    tracing::info!("MEKF: subscribed to aleph.q_hat");

    let mut tick: u64 = 0;
    loop {
        let input = sub.next().await?;
        let euler_ned = mekf_quat_to_euler_ned(input.q_hat);

        {
            let mut c = cache.lock().unwrap();
            c.roll = euler_ned[0];
            c.pitch = euler_ned[1];
            c.yaw = euler_ned[2];
            c.valid = true;
        }

        tick += 1;
        if tick % 1000 == 1 {
            tracing::info!(
                "MEKF: rpy=[{:.1},{:.1},{:.1}] deg",
                euler_ned[0].to_degrees(),
                euler_ned[1].to_degrees(),
                euler_ned[2].to_degrees(),
            );
        }
    }
}
