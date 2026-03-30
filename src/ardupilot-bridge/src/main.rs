mod ardupilot_ipc;
mod can_output;
mod config;
mod coordinate;
mod elodin;
mod hitl;
mod imu_filter;

use crate::ardupilot_ipc::{ServoOutput, SitlJsonPacket, ImuData};
use crate::can_output::CanOutput;
use crate::config::{Config, HomeLocation};
use crate::coordinate::{
    gyro_flu_to_frd, accel_flu_to_frd,
    ubx_lla_to_ned, ubx_vel_to_ms, ubx_heading_to_rad,
    mekf_quat_to_euler_ned, blend_angle,
};
use crate::elodin::{M10QInput, MekfInput, MotorTelemetry, SensorInput};
use crate::imu_filter::ConingIntegrator;

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

/// Cached filtered IMU output from the coning/sculling integrator.
/// Written by the IMU background thread at ~400 Hz (every 4 raw samples),
/// read by the main loop for JSON packet construction.
#[derive(Debug, Clone, Copy)]
struct ImuCache {
    gyro: [f64; 3],      // coning-corrected FRD rad/s
    accel: [f64; 3],     // sculling-corrected FRD m/s^2
    timestamp: f64,       // seconds since bridge start
    seq: u64,             // monotonic output sequence number (~400 Hz)
    raw_count: u64,       // total raw samples processed (~1500 Hz)
}

impl Default for ImuCache {
    fn default() -> Self {
        Self {
            gyro: [0.0; 3],
            accel: [0.0; 3],
            timestamp: 0.0,
            seq: 0,
            raw_count: 0,
        }
    }
}

/// Cached MEKF attitude, updated by the MEKF subscription task at ~500 Hz.
/// Provides ArduPilot SITL with a sensor-fused attitude reference from the
/// real BMI270 + BMM350, which drives the SITL's internal compass simulation.
#[derive(Debug, Clone, Copy)]
struct AttitudeCache {
    euler_ned: [f64; 3],  // [roll, pitch, yaw] in NED/FRD radians
    valid: bool,
}

impl Default for AttitudeCache {
    fn default() -> Self {
        Self {
            euler_ned: [0.0; 3],
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
    udp_socket
        .set_read_timeout(Some(Duration::from_millis(100)))
        .ok();

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

    // MEKF attitude subscription -- background thread, ~500 Hz.
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
            &gps_cache, &attitude_cache, &imu_cache,
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
    attitude_cache: &Arc<Mutex<AttitudeCache>>,
    imu_cache: &Arc<Mutex<ImuCache>>,
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
    let mut output_tick: u64 = 0;

    loop {
        let mut servo_buf = [0u8; 256];
        if let Ok((n, src)) = udp_socket.recv_from(&mut servo_buf) {
            if let Some(servo) = ServoOutput::from_bytes(&servo_buf[..n]) {
                if ap_addr.is_none() {
                    tracing::info!("ArduPilot connected from {}", src);
                }
                ap_addr = Some(src);

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
        last_imu_seq = imu.seq;
        output_tick += 1;

        let gps = gps_cache.lock().unwrap();
        let (position, velocity, ground_speed_ms) = if gps.has_fix {
            (gps.position_ned, gps.velocity_ned, gps.ground_speed_ms)
        } else {
            ([0.0; 3], [0.0; 3], 0.0)
        };
        let gps_yaw = gps.yaw_rad;
        let gps_fix = gps.has_fix;
        drop(gps);

        let att = attitude_cache.lock().unwrap();
        let attitude = if att.valid {
            let mekf_yaw = att.euler_ned[2];
            let yaw = if !gps_fix {
                mekf_yaw
            } else if ground_speed_ms > 3.0 {
                gps_yaw
            } else if ground_speed_ms > 1.0 {
                let alpha = (ground_speed_ms - 1.0) / 2.0;
                blend_angle(mekf_yaw, gps_yaw, alpha)
            } else {
                mekf_yaw
            };
            [att.euler_ned[0], att.euler_ned[1], yaw]
        } else if gps_fix {
            [0.0, 0.0, gps_yaw]
        } else {
            [0.0; 3]
        };
        drop(att);

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

        if output_tick % 400 == 0 {
            let att_c = attitude_cache.lock().unwrap();
            let gps_c = gps_cache.lock().unwrap();
            tracing::info!(
                "out={} raw={} ratio={:.1} gps={} sats={} att=[{:.1},{:.1},{:.1}]deg",
                output_tick,
                imu.raw_count,
                imu.raw_count as f64 / output_tick.max(1) as f64,
                gps_c.has_fix, gps_c.satellites,
                att_c.euler_ned[0].to_degrees(),
                att_c.euler_ned[1].to_degrees(),
                att_c.euler_ned[2].to_degrees(),
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Background tasks: GPS and MEKF subscriptions
// ---------------------------------------------------------------------------

/// IMU subscription task. Runs in a dedicated thread with its own stellarator
/// runtime and Elodin-DB connection. Updates the shared ImuCache at ~1500 Hz.
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

    let mut integrator = ConingIntegrator::new();
    let mut raw_count: u64 = 0;
    let mut output_seq: u64 = 0;
    let mut prev_ts = start.elapsed().as_secs_f64();

    loop {
        let input = sub.next().await?;
        raw_count += 1;
        let timestamp = start.elapsed().as_secs_f64();
        let dt = timestamp - prev_ts;
        prev_ts = timestamp;

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

        if let Some(filtered) = integrator.push(gyro_frd, accel_frd, dt) {
            output_seq += 1;
            {
                let mut c = cache.lock().unwrap();
                c.gyro = filtered.gyro;
                c.accel = filtered.accel;
                c.timestamp = timestamp;
                c.seq = output_seq;
                c.raw_count = raw_count;
            }
        }

        if raw_count % 10000 == 1 {
            tracing::info!("IMU: raw={} filtered={} ratio={:.1}",
                raw_count, output_seq,
                raw_count as f64 / output_seq.max(1) as f64);
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

/// MEKF attitude subscription task. Reconnects on error.
/// Subscribes to aleph.q_hat (quaternion from the Elodin MEKF) and converts
/// to NED Euler angles for the SITL attitude field.
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

    let mut mekf_tick: u64 = 0;
    loop {
        let mekf = sub.next().await?;
        let euler_ned = mekf_quat_to_euler_ned(mekf.q_hat);

        {
            let mut c = cache.lock().unwrap();
            c.euler_ned = euler_ned;
            c.valid = true;
        }

        mekf_tick += 1;
        if mekf_tick % 500 == 1 {
            tracing::info!(
                "MEKF: roll={:.1} pitch={:.1} yaw={:.1} deg",
                euler_ned[0].to_degrees(),
                euler_ned[1].to_degrees(),
                euler_ned[2].to_degrees(),
            );
        }
    }
}
