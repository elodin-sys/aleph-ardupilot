mod ardupilot_ipc;
mod can_output;
mod config;
mod coordinate;
mod elodin;

use crate::ardupilot_ipc::{
    AlephBaroPacket, AlephGpsPacket, AlephImuPacket, AlephMagPacket, ServoOutput,
    DEFAULT_SENSOR_PORT,
};
use crate::can_output::CanOutput;
use crate::config::Config;
use crate::coordinate::{accel_g_to_ms2, gyro_dps_to_rads, mekf_quat_to_euler, synthesize_mag_field_mgauss};
use crate::elodin::{AlephBaroInput, GPSInput, MekfInput, MotorTelemetry, SensorInput};

use anyhow::Context;
use clap::Parser;
use impeller2::types::{PacketId, Timestamp};
use impeller2_stellar::{Client, SinkExt, StreamExt};
use std::net::{IpAddr, SocketAddr, UdpSocket};
use std::sync::{Arc, Mutex};
use std::time::Duration;

fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt::init();
    stellarator::run(run)
}

const SUBSCRIBE_TIMEOUT: Duration = Duration::from_secs(5);

async fn subscribe_timeout<T, E>(
    next: impl std::future::Future<Output = Result<T, E>>,
    label: &str,
) -> anyhow::Result<T>
where
    E: std::fmt::Display,
{
    futures_lite::future::race(
        async {
            match next.await {
                Ok(v) => Ok(v),
                Err(e) => Err(anyhow::anyhow!("{label}: subscribe error: {e}")),
            }
        },
        async {
            stellarator::sleep(SUBSCRIBE_TIMEOUT).await;
            Err(anyhow::anyhow!(
                "{label}: no data within {}s of subscribing, reconnecting",
                SUBSCRIBE_TIMEOUT.as_secs(),
            ))
        },
    )
    .await
}

#[derive(Debug, Clone, Copy, Default)]
struct GpsCache {
    lat: i32,
    lon: i32,
    alt_msl: i32,
    vel_ned: [i32; 3],
    h_acc: u32,
    v_acc: u32,
    s_acc: u32,
    ground_speed: u32,
    fix_type: u8,
    satellites: u8,
    itow: u32,
    unix_epoch_ms: i64,
    epoch_us: i64,
    local_us: i64,
    seq: u64,
}

#[derive(Debug, Clone, Copy)]
struct ImuCache {
    gyro: [f32; 3],
    accel: [f32; 3],
    timestamp: f64,
    seq: u64,
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

#[derive(Debug, Clone, Copy)]
struct AttitudeCache {
    roll: f64,
    pitch: f64,
    yaw: f64,
    seq: u64,
}

impl Default for AttitudeCache {
    fn default() -> Self {
        Self {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            seq: 0,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct BaroCache {
    pressure_pa: f32,
    temperature_c: f32,
    seq: u64,
}

impl Default for BaroCache {
    fn default() -> Self {
        Self {
            pressure_pa: 0.0,
            temperature_c: 0.0,
            seq: 0,
        }
    }
}

async fn run() -> anyhow::Result<()> {
    let config = Config::parse();
    tracing::info!("ardupilot-bridge starting with config: {:?}", config);

    let ap_host_ip: IpAddr = config
        .ap_host
        .parse()
        .context("parse AP host address")?;
    let ap_sensor_addr = SocketAddr::new(ap_host_ip, DEFAULT_SENSOR_PORT);
    tracing::info!("ArduPilot sensor target: {}", ap_sensor_addr);

    let telemetry_id: PacketId = fastrand::u16(..).to_le_bytes();

    let bind_addr: SocketAddr = format!("0.0.0.0:{}", config.servo_port)
        .parse()
        .unwrap();
    let udp_socket = UdpSocket::bind(bind_addr).context("bind UDP control port")?;
    udp_socket
        .set_nonblocking(true)
        .context("set UDP non-blocking")?;

    let mut can = CanOutput::new(&config.can_interface);
    can.open().context("open CAN interface")?;

    tracing::info!(
        "bridge bound to UDP :{}, waiting for ArduPilot servo output, CAN={}",
        config.servo_port,
        if can.is_enabled() {
            &config.can_interface
        } else {
            "disabled"
        }
    );

    let gps_cache = Arc::new(Mutex::new(GpsCache::default()));
    {
        let gps_cache = Arc::clone(&gps_cache);
        let elodin_addr_str = config.elodin_addr.clone();
        std::thread::spawn(move || {
            if let Err(e) = stellarator::run(move || gps_task(elodin_addr_str, gps_cache)) {
                tracing::error!("GPS task fatal: {:?}", e);
            }
        });
    }

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

    let baro_cache = Arc::new(Mutex::new(BaroCache::default()));
    {
        let baro_cache = Arc::clone(&baro_cache);
        let elodin_addr_str = config.elodin_addr.clone();
        std::thread::spawn(move || {
            if let Err(e) = stellarator::run(move || baro_task(elodin_addr_str, baro_cache)) {
                tracing::error!("Baro task fatal: {:?}", e);
            }
        });
    }

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

    loop {
        if let Err(err) = bridge_loop(
            &config,
            telemetry_id,
            &udp_socket,
            &mut can,
            &gps_cache,
            &attitude_cache,
            &baro_cache,
            &imu_cache,
            ap_sensor_addr,
        )
        .await
        {
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
    baro_cache: &Arc<Mutex<BaroCache>>,
    imu_cache: &Arc<Mutex<ImuCache>>,
    ap_sensor_addr: SocketAddr,
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

    let mut saw_ap_servo = false;
    let mut last_imu_seq: u64 = 0;
    let mut last_gps_seq_sent: u64 = 0;
    let mut last_mag_seq_sent: u64 = 0;
    let mut last_baro_seq_sent: u64 = 0;
    let mut last_baro_fallback_gps_seq_sent: u64 = 0;

    let mut rate_timer = std::time::Instant::now();
    let mut imu_sent: u64 = 0;
    let mut gps_sent: u64 = 0;
    let mut mag_sent: u64 = 0;
    let mut baro_sent: u64 = 0;
    let mut servo_recv: u64 = 0;
    let mut imu_skipped: u64 = 0;
    let mut loop_iters: u64 = 0;
    let mut bin_ser_total_us: u64 = 0;
    let mut bin_ser_count: u64 = 0;
    let mut roundtrip_total_us: u64 = 0;
    let mut roundtrip_count: u64 = 0;
    let mut last_sensor_send_at: Option<std::time::Instant> = None;

    loop {
        loop_iters += 1;
        let mut did_work = false;

        let mut servo_buf = [0u8; 256];
        if let Ok((n, src)) = udp_socket.recv_from(&mut servo_buf) {
            if let Some(servo) = ServoOutput::from_bytes(&servo_buf[..n]) {
                if !saw_ap_servo {
                    tracing::info!("ArduPilot connected from {}", src);
                }
                saw_ap_servo = true;
                servo_recv += 1;
                if let Some(send_at) = last_sensor_send_at.take() {
                    let rtt_us = send_at.elapsed().as_micros() as u64;
                    roundtrip_total_us = roundtrip_total_us.saturating_add(rtt_us);
                    roundtrip_count = roundtrip_count.saturating_add(1);
                }

                let motors_norm = servo.motors_normalized(config.num_motors);
                let motor_pwm: [u16; 4] = [servo.pwm[0], servo.pwm[1], servo.pwm[2], servo.pwm[3]];
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
                did_work = true;
            }
        }

        let imu = {
            let c = imu_cache.lock().unwrap();
            *c
        };

        if imu.seq > 0 && imu.seq != last_imu_seq {
            let skipped = imu.seq.saturating_sub(last_imu_seq).saturating_sub(1);
            imu_skipped += skipped;
            last_imu_seq = imu.seq;

            let packet = AlephImuPacket {
                gyro: imu.gyro,
                accel_body: imu.accel,
            };
            let send_started_at = std::time::Instant::now();
            let bytes = packet.to_bytes();
            udp_socket
                .send_to(&bytes, ap_sensor_addr)
                .context("send IMU packet to ArduPilot")?;
            let ser_us = send_started_at.elapsed().as_micros() as u64;
            bin_ser_total_us = bin_ser_total_us.saturating_add(ser_us);
            bin_ser_count = bin_ser_count.saturating_add(1);
            last_sensor_send_at = Some(send_started_at);
            imu_sent += 1;
            did_work = true;
        }

        let gps = {
            let c = gps_cache.lock().unwrap();
            *c
        };
        if gps.seq > 0 && gps.seq != last_gps_seq_sent {
            last_gps_seq_sent = gps.seq;
            let packet = AlephGpsPacket {
                lat: gps.lat,
                lon: gps.lon,
                alt_msl: gps.alt_msl,
                vel_ned: gps.vel_ned,
                h_acc: gps.h_acc,
                v_acc: gps.v_acc,
                s_acc: gps.s_acc,
                ground_speed: gps.ground_speed,
                fix_type: gps.fix_type,
                satellites: gps.satellites,
                itow: gps.itow,
                unix_epoch_ms: gps.unix_epoch_ms,
            };
            let send_started_at = std::time::Instant::now();
            let bytes = packet.to_bytes();
            udp_socket
                .send_to(&bytes, ap_sensor_addr)
                .context("send GPS packet to ArduPilot")?;
            let ser_us = send_started_at.elapsed().as_micros() as u64;
            bin_ser_total_us = bin_ser_total_us.saturating_add(ser_us);
            bin_ser_count = bin_ser_count.saturating_add(1);
            gps_sent += 1;
            did_work = true;
        }

        let att = {
            let c = attitude_cache.lock().unwrap();
            *c
        };
        if att.seq > 0 && att.seq != last_mag_seq_sent {
            last_mag_seq_sent = att.seq;
            let synth = synthesize_mag_field_mgauss(att.roll, att.pitch, att.yaw);
            let packet = AlephMagPacket {
                mag_mgauss: synth,
            };
            let send_started_at = std::time::Instant::now();
            let bytes = packet.to_bytes();
            udp_socket
                .send_to(&bytes, ap_sensor_addr)
                .context("send synthesized compass packet to ArduPilot")?;
            let ser_us = send_started_at.elapsed().as_micros() as u64;
            bin_ser_total_us = bin_ser_total_us.saturating_add(ser_us);
            bin_ser_count = bin_ser_count.saturating_add(1);
            last_sensor_send_at = Some(send_started_at);
            mag_sent += 1;
            did_work = true;
        }

        let baro = {
            let c = baro_cache.lock().unwrap();
            *c
        };
        if baro.seq > 0 && baro.seq != last_baro_seq_sent {
            last_baro_seq_sent = baro.seq;
            let packet = AlephBaroPacket {
                pressure_pa: baro.pressure_pa,
                temperature_c: baro.temperature_c,
            };
            let send_started_at = std::time::Instant::now();
            let bytes = packet.to_bytes();
            udp_socket
                .send_to(&bytes, ap_sensor_addr)
                .context("send barometer packet to ArduPilot")?;
            let ser_us = send_started_at.elapsed().as_micros() as u64;
            bin_ser_total_us = bin_ser_total_us.saturating_add(ser_us);
            bin_ser_count = bin_ser_count.saturating_add(1);
            last_sensor_send_at = Some(send_started_at);
            baro_sent += 1;
            did_work = true;
        } else if baro.seq == 0
            && gps.seq > 0
            && gps.seq == last_gps_seq_sent
            && gps.seq != last_baro_fallback_gps_seq_sent
        {
            // Fallback only while aleph baro stream has not started.
            let alt_m = gps.alt_msl as f64 * 1e-3;
            let pressure = (101325.0 * (1.0 - 2.2558e-5 * alt_m).powf(5.2559)) as f32;
            let packet = AlephBaroPacket {
                pressure_pa: pressure,
                temperature_c: 39.8,
            };
            let send_started_at = std::time::Instant::now();
            let bytes = packet.to_bytes();
            udp_socket
                .send_to(&bytes, ap_sensor_addr)
                .context("send fallback barometer packet to ArduPilot")?;
            let ser_us = send_started_at.elapsed().as_micros() as u64;
            bin_ser_total_us = bin_ser_total_us.saturating_add(ser_us);
            bin_ser_count = bin_ser_count.saturating_add(1);
            last_sensor_send_at = Some(send_started_at);
            last_baro_fallback_gps_seq_sent = gps.seq;
            baro_sent += 1;
            did_work = true;
        }

        let elapsed = rate_timer.elapsed();
        if elapsed >= Duration::from_secs(1) {
            let secs = elapsed.as_secs_f64();
            let imu_db_hz = {
                let c = imu_cache.lock().unwrap();
                c.seq as f64 / c.timestamp.max(0.001)
            };
            let bin_total = imu_sent + gps_sent + mag_sent + baro_sent;
            tracing::info!(
                "RATES: imu_from_db={:.0}Hz bin_to_ap={:.0}Hz imu_to_ap={:.0}Hz gps_to_ap={:.0}Hz mag_to_ap={:.0}Hz baro_to_ap={:.0}Hz servo_from_ap={:.0}Hz imu_skipped={} loop={:.0}/s",
                imu_db_hz,
                bin_total as f64 / secs,
                imu_sent as f64 / secs,
                gps_sent as f64 / secs,
                mag_sent as f64 / secs,
                baro_sent as f64 / secs,
                servo_recv as f64 / secs,
                imu_skipped,
                loop_iters as f64 / secs,
            );
            let bin_ser_avg_us = bin_ser_total_us as f64 / bin_ser_count.max(1) as f64;
            let roundtrip_avg_us = roundtrip_total_us as f64 / roundtrip_count.max(1) as f64;
            tracing::info!(
                "PERF: bin_ser_avg={:.0}us roundtrip_avg={:.0}us roundtrip_count={}",
                bin_ser_avg_us,
                roundtrip_avg_us,
                roundtrip_count
            );

            rate_timer = std::time::Instant::now();
            imu_sent = 0;
            gps_sent = 0;
            mag_sent = 0;
            baro_sent = 0;
            servo_recv = 0;
            imu_skipped = 0;
            loop_iters = 0;
            bin_ser_total_us = 0;
            bin_ser_count = 0;
            roundtrip_total_us = 0;
            roundtrip_count = 0;
        }

        if !did_work {
            stellarator::sleep(Duration::from_micros(500)).await;
        }
    }
}

async fn imu_task(elodin_addr: String, cache: Arc<Mutex<ImuCache>>) -> anyhow::Result<()> {
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

    let first = subscribe_timeout(sub.next(), "IMU").await?;
    let mut seq: u64 = 1;
    let timestamp = start.elapsed().as_secs_f64();
    let gyro = gyro_dps_to_rads([first.gyro[0] as f64, first.gyro[1] as f64, first.gyro[2] as f64]);
    let accel = accel_g_to_ms2([first.accel[0] as f64, first.accel[1] as f64, first.accel[2] as f64]);
    {
        let mut c = cache.lock().unwrap();
        c.gyro = [gyro[0] as f32, gyro[1] as f32, gyro[2] as f32];
        c.accel = [accel[0] as f32, accel[1] as f32, accel[2] as f32];
        c.timestamp = timestamp;
        c.seq = seq;
    }

    loop {
        let input = sub.next().await?;
        seq += 1;
        let timestamp = start.elapsed().as_secs_f64();
        let gyro = gyro_dps_to_rads([
            input.gyro[0] as f64,
            input.gyro[1] as f64,
            input.gyro[2] as f64,
        ]);
        let accel = accel_g_to_ms2([
            input.accel[0] as f64,
            input.accel[1] as f64,
            input.accel[2] as f64,
        ]);

        {
            let mut c = cache.lock().unwrap();
            c.gyro = [gyro[0] as f32, gyro[1] as f32, gyro[2] as f32];
            c.accel = [accel[0] as f32, accel[1] as f32, accel[2] as f32];
            c.timestamp = timestamp;
            c.seq = seq;
        }

        if seq % 10000 == 1 {
            let hz = seq as f64 / timestamp.max(0.001);
            tracing::info!("IMU: samples={} rate={:.0} Hz", seq, hz);
        }
    }
}

async fn gps_task(elodin_addr: String, cache: Arc<Mutex<GpsCache>>) -> anyhow::Result<()> {
    loop {
        match gps_subscribe_loop(&elodin_addr, &cache).await {
            Ok(()) => {}
            Err(e) => {
                tracing::warn!("GPS subscription error (will retry): {}", e);
                stellarator::sleep(Duration::from_secs(2)).await;
            }
        }
    }
}

async fn gps_subscribe_loop(elodin_addr: &str, cache: &Arc<Mutex<GpsCache>>) -> anyhow::Result<()> {
    let addr: SocketAddr = elodin_addr.parse().context("parse elodin addr for GPS")?;
    let mut client = Client::connect(addr).await.map_err(anyhow::Error::from)?;
    let mut sub = client.subscribe::<GPSInput>().await?;
    tracing::info!("GPS: subscribed to GPS vtable");

    subscribe_timeout(sub.next(), "GPS").await?;

    let mut seq: u64 = 1;
    let mut gps_tick: u64 = 1;
    loop {
        let gps = sub.next().await?;
        {
            let mut c = cache.lock().unwrap();
            c.lat = gps.lat;
            c.lon = gps.lon;
            c.alt_msl = gps.alt_msl;
            c.vel_ned = gps.vel_ned;
            c.h_acc = gps.h_acc;
            c.v_acc = gps.v_acc;
            c.s_acc = gps.s_acc;
            c.ground_speed = gps.ground_speed;
            c.fix_type = gps.fix_type;
            c.satellites = gps.satellites;
            c.itow = gps.itow;
            c.unix_epoch_ms = gps.unix_epoch_ms;
            if gps.unix_epoch_ms > 0 {
                c.epoch_us = gps.unix_epoch_ms.saturating_mul(1000);
                c.local_us = Timestamp::now().0;
            }
            c.seq = seq;
        }

        seq += 1;
        gps_tick += 1;
        if gps_tick % 10 == 1 {
            tracing::info!(
                "GPS: fix={} sats={} lat={:.7} lon={:.7} alt={:.1}m h_acc={:.2}m vel=[{:.2},{:.2},{:.2}]m/s",
                gps.fix_type,
                gps.satellites,
                gps.lat as f64 * 1e-7,
                gps.lon as f64 * 1e-7,
                gps.alt_msl as f64 * 1e-3,
                gps.h_acc as f64 * 1e-3,
                gps.vel_ned[0] as f64 * 1e-3,
                gps.vel_ned[1] as f64 * 1e-3,
                gps.vel_ned[2] as f64 * 1e-3,
            );
        }
    }
}

async fn mekf_task(elodin_addr: String, cache: Arc<Mutex<AttitudeCache>>) -> anyhow::Result<()> {
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
    let addr: SocketAddr = elodin_addr
        .parse()
        .context("parse elodin addr for MEKF")?;
    let mut client = Client::connect(addr).await.map_err(anyhow::Error::from)?;
    let mut sub = client.subscribe::<MekfInput>().await?;
    tracing::info!("MEKF: subscribed to aleph.q_hat");

    subscribe_timeout(sub.next(), "MEKF").await?;

    let mut seq: u64 = 1;
    let mut tick: u64 = 1;
    loop {
        let input = sub.next().await?;
        let euler = mekf_quat_to_euler(input.q_hat);

        {
            let mut c = cache.lock().unwrap();
            c.roll = euler[0];
            c.pitch = euler[1];
            c.yaw = euler[2];
            c.seq = seq;
        }

        seq += 1;
        tick += 1;
        if tick % 1000 == 1 {
            tracing::info!(
                "MEKF: rpy=[{:.1},{:.1},{:.1}] deg",
                euler[0].to_degrees(),
                euler[1].to_degrees(),
                euler[2].to_degrees(),
            );
        }
    }
}

async fn baro_task(elodin_addr: String, cache: Arc<Mutex<BaroCache>>) -> anyhow::Result<()> {
    loop {
        match baro_subscribe_loop(&elodin_addr, &cache).await {
            Ok(()) => {}
            Err(e) => {
                tracing::warn!("Baro subscription error (will retry): {}", e);
                stellarator::sleep(Duration::from_secs(2)).await;
            }
        }
    }
}

async fn baro_subscribe_loop(elodin_addr: &str, cache: &Arc<Mutex<BaroCache>>) -> anyhow::Result<()> {
    let addr: SocketAddr = elodin_addr.parse().context("parse elodin addr for baro")?;
    let mut client = Client::connect(addr).await.map_err(anyhow::Error::from)?;
    let mut sub = client.subscribe::<AlephBaroInput>().await?;
    tracing::info!("Baro: subscribed to aleph baro vtable (f32)");

    let first = subscribe_timeout(sub.next(), "Baro").await?;
    let mut seq: u64 = 1;
    {
        let mut c = cache.lock().unwrap();
        c.pressure_pa = first.baro;
        c.temperature_c = first.baro_temp;
        c.seq = seq;
    }

    let mut tick: u64 = 1;
    loop {
        let input = sub.next().await?;
        seq += 1;
        let pressure_pa = input.baro;
        let temperature_c = input.baro_temp;
        {
            let mut c = cache.lock().unwrap();
            c.pressure_pa = pressure_pa;
            c.temperature_c = temperature_c;
            c.seq = seq;
        }
        tick += 1;
        if tick % 20 == 1 {
            tracing::info!(
                "BARO: pressure={:.1}Pa temp={:.2}C",
                pressure_pa,
                temperature_c,
            );
        }
    }
}

