"""ArduPilot sim-HITL simulation using Elodin physics.

The ardupilot-bridge (running on the Aleph) connects to this simulation's
Elodin-DB.  Data flows entirely through the DB -- the same code path as
real hardware:

    Simulation  -->  "imu" entity (gyro/accel/mag f32)      -->  bridge subscribes
    Simulation  -->  "ublox" entity (GPS UBX integers)      -->  bridge subscribes
    Simulation  -->  "aleph" entity (q_hat/baro/baro_temp) -->  bridge subscribes
    bridge writes "ardupilot" entity (motor_command)       -->  Simulation reads

The bridge is completely unaware a simulation is involved -- it sees the
exact same DB schema as when real hardware sensors are writing.

Usage:
    elodin editor sim/sim-hitl/main.py   # with 3D visualisation
    elodin run sim/sim-hitl/main.py      # headless
"""

import math
import os
import shutil
import time
import typing as ty
from dataclasses import dataclass, field

import elodin as el
import jax
import jax.numpy as jnp
import numpy as np

from config import DEFAULT_CONFIG, Config
from gps import GPS, WGS84_A, gps as gps_system
from sensors import IMU
from sim import Drone, system

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

config = DEFAULT_CONFIG
config.set_as_global()

DB_PATH = "/tmp/sim-hitl-db"

# ---------------------------------------------------------------------------
# Bridge-facing output schemas (must match ardupilot-bridge Rust structs)
# ---------------------------------------------------------------------------

_EXT = {"external_control": "true"}

SensorMag = ty.Annotated[
    jax.Array,
    el.Component("mag", el.ComponentType(el.PrimitiveType.F32, (3,)), metadata=_EXT),
]
SensorGyro = ty.Annotated[
    jax.Array,
    el.Component("gyro", el.ComponentType(el.PrimitiveType.F32, (3,)), metadata=_EXT),
]
SensorAccel = ty.Annotated[
    jax.Array,
    el.Component("accel", el.ComponentType(el.PrimitiveType.F32, (3,)), metadata=_EXT),
]

GPSLat = ty.Annotated[
    jax.Array,
    el.Component("lat", el.ComponentType(el.PrimitiveType.I32, (1,)), metadata=_EXT),
]
GPSLon = ty.Annotated[
    jax.Array,
    el.Component("lon", el.ComponentType(el.PrimitiveType.I32, (1,)), metadata=_EXT),
]
GPSAltMsl = ty.Annotated[
    jax.Array,
    el.Component("alt_msl", el.ComponentType(el.PrimitiveType.I32, (1,)), metadata=_EXT),
]
GPSAltWgs84 = ty.Annotated[
    jax.Array,
    el.Component("alt_wgs84", el.ComponentType(el.PrimitiveType.I32, (1,)), metadata=_EXT),
]
GPSVelNed = ty.Annotated[
    jax.Array,
    el.Component("vel_ned", el.ComponentType(el.PrimitiveType.I32, (3,)), metadata=_EXT),
]
GPSFixType = ty.Annotated[
    jax.Array,
    el.Component("fix_type", el.ComponentType(el.PrimitiveType.U8, (1,)), metadata=_EXT),
]
GPSSatellites = ty.Annotated[
    jax.Array,
    el.Component("satellites", el.ComponentType(el.PrimitiveType.U8, (1,)), metadata=_EXT),
]
GPSHAcc = ty.Annotated[
    jax.Array,
    el.Component("h_acc", el.ComponentType(el.PrimitiveType.U32, (1,)), metadata=_EXT),
]
GPSVAcc = ty.Annotated[
    jax.Array,
    el.Component("v_acc", el.ComponentType(el.PrimitiveType.U32, (1,)), metadata=_EXT),
]
GPSSAcc = ty.Annotated[
    jax.Array,
    el.Component("s_acc", el.ComponentType(el.PrimitiveType.U32, (1,)), metadata=_EXT),
]
GPSGroundSpeed = ty.Annotated[
    jax.Array,
    el.Component("ground_speed", el.ComponentType(el.PrimitiveType.U32, (1,)), metadata=_EXT),
]
GPSHeadingMotion = ty.Annotated[
    jax.Array,
    el.Component("heading_motion", el.ComponentType(el.PrimitiveType.I32, (1,)), metadata=_EXT),
]
GPSValidFlags = ty.Annotated[
    jax.Array,
    el.Component("valid_flags", el.ComponentType(el.PrimitiveType.U8, (1,)), metadata=_EXT),
]
GPSItow = ty.Annotated[
    jax.Array,
    el.Component("itow", el.ComponentType(el.PrimitiveType.U32, (1,)), metadata=_EXT),
]
GPSUnixEpochMs = ty.Annotated[
    jax.Array,
    el.Component("unix_epoch_ms", el.ComponentType(el.PrimitiveType.I64, (1,)), metadata=_EXT),
]


@dataclass
class SensorOutput(el.Archetype):
    mag: SensorMag = field(default_factory=lambda: jnp.zeros(3, dtype=jnp.float32))
    gyro: SensorGyro = field(default_factory=lambda: jnp.zeros(3, dtype=jnp.float32))
    accel: SensorAccel = field(default_factory=lambda: jnp.zeros(3, dtype=jnp.float32))


@dataclass
class GPSOutput(el.Archetype):
    lat: GPSLat = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.int32))
    lon: GPSLon = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.int32))
    alt_msl: GPSAltMsl = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.int32))
    alt_wgs84: GPSAltWgs84 = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.int32))
    vel_ned: GPSVelNed = field(default_factory=lambda: jnp.zeros(3, dtype=jnp.int32))
    fix_type: GPSFixType = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.uint8))
    satellites: GPSSatellites = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.uint8))
    h_acc: GPSHAcc = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.uint32))
    v_acc: GPSVAcc = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.uint32))
    s_acc: GPSSAcc = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.uint32))
    ground_speed: GPSGroundSpeed = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.uint32))
    heading_motion: GPSHeadingMotion = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.int32))
    valid_flags: GPSValidFlags = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.uint8))
    itow: GPSItow = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.uint32))
    unix_epoch_ms: GPSUnixEpochMs = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.int64))

AlephQHat = ty.Annotated[
    jax.Array,
    el.Component("q_hat", el.ComponentType(el.PrimitiveType.F64, (4,)), metadata=_EXT),
]
AlephBaro = ty.Annotated[
    jax.Array,
    el.Component("baro", el.ComponentType.F64, metadata=_EXT),
]
AlephBaroTemp = ty.Annotated[
    jax.Array,
    el.Component("baro_temp", el.ComponentType.F64, metadata=_EXT),
]


@dataclass
class AlephOutput(el.Archetype):
    q_hat: AlephQHat = field(default_factory=lambda: jnp.array([0.0, 0.0, 0.0, 1.0]))
    baro: AlephBaro = field(default_factory=lambda: jnp.float64(101325.0))
    baro_temp: AlephBaroTemp = field(default_factory=lambda: jnp.float64(39.8))


# ---------------------------------------------------------------------------
# World
# ---------------------------------------------------------------------------

world = el.World()

drone = world.spawn(
    [
        el.Body(
            world_pos=config.spatial_transform,
            inertia=config.spatial_inertia,
        ),
        Drone(),
        IMU(),
        GPS(),
    ],
    name="drone",
)

# Interface entities matching the bridge's VTable namespaces.
# "imu"       -- gyro/accel/mag in f32 (BMM350 on-board mag)
# "ublox"      -- GPS in UBX integer format
# "aleph"     -- MEKF attitude + baro
# "ardupilot" -- motor data the bridge writes back
imu = world.spawn([SensorOutput()], name="imu")
ublox = world.spawn([GPSOutput()], name="ublox")
aleph = world.spawn([AlephOutput()], name="aleph")
ardupilot = world.spawn([], name="ardupilot")

# ---------------------------------------------------------------------------
# Editor schematic
# ---------------------------------------------------------------------------

world.schematic(
    """
    tabs {
        hsplit name="Viewport" {
            viewport name=Viewport pos="drone.world_pos + (0,0,0,0, 2,2,2)" look_at="drone.world_pos" show_grid=#true active=#true
            vsplit share=0.4 {
                graph "drone.motor_command" name="Motor Commands (physics input)"
                graph "ardupilot.motor_command" name="Motor Commands (from bridge)"
                graph "drone.thrust" name="Motor Thrust (N)"
                graph "drone.motor_rpm" name="Motor RPM"
            }
        }
        vsplit name="Sensors" {
            graph "drone.sim_gyro" name="Gyroscope f64 (physics)"
            graph "IMU.gyro" name="Gyroscope f32 (to bridge)"
            graph "drone.sim_accel" name="Accelerometer (physics)"
            graph "drone.magnetometer" name="Magnetometer (physics)"
        }
        vsplit name="GPS" {
            graph "drone.gps_lat" name="GPS Latitude (deg)"
            graph "drone.gps_lon" name="GPS Longitude (deg)"
            graph "drone.gps_alt_msl" name="GPS Altitude MSL (m)"
            graph "drone.gps_vel_ned" name="GPS Velocity NED (m/s)"
            graph "drone.gps_ground_speed" name="GPS Ground Speed (m/s)"
        }
        vsplit name="State" {
            graph "drone.world_pos.linear()" name="Position (ENU m)"
            graph "drone.world_vel.linear()" name="Velocity (m/s)"
            graph "drone.body_thrust" name="Body Thrust"
        }
    }

    object_3d drone.world_pos {
        glb path="https://assets.elodin.systems/assets/edu-450-v2-drone.glb"
        icon builtin="flight" {
            visibility_range min=500.0
            color 0 188 212
        }
    }

    vector_arrow "(1,0,0)" origin="drone.world_pos" scale=1.0 name="X" body_frame=#true { color red 150 }
    vector_arrow "(0,1,0)" origin="drone.world_pos" scale=1.0 name="Y" body_frame=#true { color green 150 }
    vector_arrow "(0,0,1)" origin="drone.world_pos" scale=1.0 name="Z" body_frame=#true { color blue 150 }

    line_3d drone.world_pos line_width=2.0 perspective=#false { color yolk }
    """,
    "sim-hitl.kdl",
)

# ---------------------------------------------------------------------------
# Post-step: bridge data between physics and interface entities
# ---------------------------------------------------------------------------

_last_print = [0.0]
_start_time = [None]
_gps_rng = np.random.default_rng(42)
_baro_rng = np.random.default_rng(99)
_home_lat_rad = math.radians(config.home_lat)

_mag_interval = max(1, round(0.01 / config.dt))     # ~100 Hz
_baro_interval = max(1, round(1.0 / (10.0 * config.dt)))  # ~10 Hz

_MOTOR_ZEROS = np.zeros(4, dtype=np.float64)


def post_step(tick: int, ctx: el.StepContext):
    """Bridge sensor data and motor commands using batched DB operations."""
    if _start_time[0] is None:
        _start_time[0] = time.time()

    t = tick * config.dt
    do_mag = tick % _mag_interval == 0
    do_baro = tick % _baro_interval == 0

    # ------------------------------------------------------------------
    # Batched read: one DB lock acquisition for all component reads
    # ------------------------------------------------------------------
    reads = ["drone.sim_gyro", "drone.sim_accel", "drone.world_pos",
             "drone.gps_set", "ardupilot.motor_command", "ardupilot.motor_pwm"]
    if do_mag:
        reads.append("drone.magnetometer")

    try:
        data = ctx.component_batch_operation(reads=reads)
    except (RuntimeError, ValueError):
        return

    # ------------------------------------------------------------------
    # Process sensor data (pure Python, no FFI)
    # ------------------------------------------------------------------
    gyro = np.array(data["drone.sim_gyro"], dtype=np.float32)
    accel = np.array(data["drone.sim_accel"], dtype=np.float32) / 9.80665

    world_pos = np.array(data["drone.world_pos"], dtype=np.float64)
    q_hat = world_pos[:4]

    writes: dict[str, np.ndarray] = {
        "imu.gyro": gyro,
        "imu.accel": accel,
        "aleph.q_hat": q_hat,
    }

    # Mag at ~100 Hz
    if do_mag:
        writes["imu.mag"] = np.array(data["drone.magnetometer"], dtype=np.float32)

    # Baro at ~10 Hz
    if do_baro:
        alt_m = float(world_pos[6]) + config.home_alt
        pressure = 101325.0 * (1.0 - 2.2558e-5 * alt_m) ** 5.2559
        pressure += _baro_rng.normal(0, 0.5)
        temp = 39.8 + _baro_rng.normal(0, 0.01)
        writes["aleph.baro"] = np.array([pressure], dtype=np.float64)
        writes["aleph.baro_temp"] = np.array([temp], dtype=np.float64)

    # GPS at ~5 Hz (gated by physics gps_set flag)
    gps_set_val = int(np.array(data["drone.gps_set"]).flat[0])
    if gps_set_val:
        gps_reads = ctx.component_batch_operation(
            reads=["drone.gps_lat", "drone.gps_lon",
                   "drone.gps_alt_msl", "drone.gps_vel_ned"])

        lat_truth = float(np.array(gps_reads["drone.gps_lat"]).flat[0])
        lon_truth = float(np.array(gps_reads["drone.gps_lon"]).flat[0])
        alt_truth = float(np.array(gps_reads["drone.gps_alt_msl"]).flat[0])
        vel_truth = np.array(gps_reads["drone.gps_vel_ned"], dtype=np.float64).flatten()

        pos_noise_m = _gps_rng.normal(0, config.gps_hacc_std, size=2)
        alt_noise_m = _gps_rng.normal(0, config.gps_vacc_std)
        vel_noise_ms = _gps_rng.normal(0, config.gps_sacc_std, size=3)

        lat = lat_truth + pos_noise_m[0] / WGS84_A * (180.0 / math.pi)
        lon = lon_truth + pos_noise_m[1] / (WGS84_A * math.cos(_home_lat_rad)) * (180.0 / math.pi)
        alt = alt_truth + alt_noise_m
        vel = vel_truth + vel_noise_ms

        gs = math.sqrt(vel[0] ** 2 + vel[1] ** 2)
        hdg = math.degrees(math.atan2(vel[1], vel[0]))
        if hdg < 0:
            hdg += 360.0

        lat_e7 = int(round(lat * 1e7))
        lon_e7 = int(round(lon * 1e7))
        alt_mm = int(round(alt * 1e3))
        vel_ned_mms = [int(round(v * 1e3)) for v in vel]
        gs_mms = int(round(gs * 1e3))
        hdg_e5 = int(round(hdg * 1e5))
        h_acc_mm = int(round(config.gps_hacc_std * 1e3))
        v_acc_mm = int(round(config.gps_vacc_std * 1e3))
        s_acc_mms = int(round(config.gps_sacc_std * 1e3))

        writes.update({
            "ublox.lat": np.array([lat_e7], dtype=np.int32),
            "ublox.lon": np.array([lon_e7], dtype=np.int32),
            "ublox.alt_msl": np.array([alt_mm], dtype=np.int32),
            "ublox.alt_wgs84": np.array([alt_mm], dtype=np.int32),
            "ublox.vel_ned": np.array(vel_ned_mms, dtype=np.int32),
            "ublox.fix_type": np.array([3], dtype=np.uint8),
            "ublox.satellites": np.array([12], dtype=np.uint8),
            "ublox.h_acc": np.array([h_acc_mm], dtype=np.uint32),
            "ublox.v_acc": np.array([v_acc_mm], dtype=np.uint32),
            "ublox.s_acc": np.array([s_acc_mms], dtype=np.uint32),
            "ublox.ground_speed": np.array([gs_mms], dtype=np.uint32),
            "ublox.heading_motion": np.array([hdg_e5], dtype=np.int32),
            "ublox.valid_flags": np.array([0x37], dtype=np.uint8),
            "ublox.itow": np.array([int(t * 1000) % 604800000], dtype=np.uint32),
            "ublox.unix_epoch_ms": np.array([int(t * 1000)], dtype=np.int64),
        })

    # Motors: ardupilot -> drone
    try:
        motor_cmd = np.array(data["ardupilot.motor_command"]).astype(np.float64)
    except (KeyError, RuntimeError):
        motor_cmd = _MOTOR_ZEROS
    try:
        motor_pwm = np.array(data["ardupilot.motor_pwm"]).astype(np.float64)
    except (KeyError, RuntimeError):
        motor_pwm = _MOTOR_ZEROS

    writes["drone.motor_command"] = motor_cmd
    writes["drone.motor_pwm"] = motor_pwm

    # ------------------------------------------------------------------
    # Batched write: one DB lock acquisition for all component writes
    # ------------------------------------------------------------------
    try:
        ctx.component_batch_operation(writes=writes)
    except (RuntimeError, ValueError):
        pass

    # Periodic status (uses its own reads to avoid bloating the hot path)
    if t - _last_print[0] >= 1.0:
        _last_print[0] = t
        elapsed = time.time() - _start_time[0]
        rtf = t / elapsed if elapsed > 0 else 0
        z = float(world_pos[6]) if len(world_pos) > 6 else 0.0
        print(
            f"  t={t:5.1f}s | z={z:+.2f}m | "
            f"motors=[{motor_cmd[0]:.3f},{motor_cmd[1]:.3f},{motor_cmd[2]:.3f},{motor_cmd[3]:.3f}] | "
            f"{rtf:.1f}x RT"
        )


# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------

sim_system = system() | gps_system

print("ArduPilot sim-HITL Simulation")
print("=============================")
print(f"Mass: {config.mass:.1f} kg")
print(f"Simulation rate: {config.simulation_rate:.0f} Hz")
print(f"GPS: {config.gps_rate:.0f} Hz, boot delay {config.gps_boot_delay:.0f}s, "
      f"hacc {config.gps_hacc_std:.1f}m, vacc {config.gps_vacc_std:.1f}m")
print(f"Home: {config.home_lat:.4f}, {config.home_lon:.4f}, {config.home_alt:.0f}m")
print(f"Duration: {config.simulation_time:.0f} s")
print(f"DB path: {DB_PATH}")
print()
print("Data flows through the DB -- same code path as real hardware.")
print('  Bridge reads: "imu" entity (gyro/accel/mag)')
print('  Bridge reads: "ublox" entity (GPS UBX integers)')
print('  Bridge reads: "aleph" entity (q_hat/baro/baro_temp)')
print('  Bridge writes: "ardupilot" entity (motor_command/motor_pwm)')
print()
print("Deploy sim-hitl config to the Aleph:")
print("  ./deploy.sh -c sim-hitl -h <aleph-ip> -u root")
print()

if os.path.exists(DB_PATH):
    shutil.rmtree(DB_PATH)

world.run(
    sim_system,
    simulation_rate=config.simulation_rate,
    generate_real_time=True,
    post_step=post_step,
    db_path=DB_PATH,
)
