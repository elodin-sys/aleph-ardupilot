"""ArduPilot sim-HITL simulation using Elodin physics.

The ardupilot-bridge (running on the Aleph) connects to this simulation's
Elodin-DB.  Data flows entirely through the DB -- the same code path as
real hardware:

    Simulation  -->  "IMU" entity (gyro/accel/mag f32)   -->  bridge subscribes
    Simulation  -->  "M10Q" entity (GPS UBX integers)    -->  bridge subscribes
    bridge writes "ardupilot" entity (motor_command)     -->  Simulation reads

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

SensorMag = ty.Annotated[
    jax.Array,
    el.Component("mag", el.ComponentType(el.PrimitiveType.F32, (3,))),
]
SensorGyro = ty.Annotated[
    jax.Array,
    el.Component("gyro", el.ComponentType(el.PrimitiveType.F32, (3,))),
]
SensorAccel = ty.Annotated[
    jax.Array,
    el.Component("accel", el.ComponentType(el.PrimitiveType.F32, (3,))),
]

M10QLat = ty.Annotated[
    jax.Array,
    el.Component("lat", el.ComponentType(el.PrimitiveType.I32, (1,))),
]
M10QLon = ty.Annotated[
    jax.Array,
    el.Component("lon", el.ComponentType(el.PrimitiveType.I32, (1,))),
]
M10QAltMsl = ty.Annotated[
    jax.Array,
    el.Component("alt_msl", el.ComponentType(el.PrimitiveType.I32, (1,))),
]
M10QAltWgs84 = ty.Annotated[
    jax.Array,
    el.Component("alt_wgs84", el.ComponentType(el.PrimitiveType.I32, (1,))),
]
M10QVelNed = ty.Annotated[
    jax.Array,
    el.Component("vel_ned", el.ComponentType(el.PrimitiveType.I32, (3,))),
]
M10QFixType = ty.Annotated[
    jax.Array,
    el.Component("fix_type", el.ComponentType(el.PrimitiveType.U8, (1,))),
]
M10QSatellites = ty.Annotated[
    jax.Array,
    el.Component("satellites", el.ComponentType(el.PrimitiveType.U8, (1,))),
]
M10QHAcc = ty.Annotated[
    jax.Array,
    el.Component("h_acc", el.ComponentType(el.PrimitiveType.U32, (1,))),
]
M10QVAcc = ty.Annotated[
    jax.Array,
    el.Component("v_acc", el.ComponentType(el.PrimitiveType.U32, (1,))),
]
M10QSAcc = ty.Annotated[
    jax.Array,
    el.Component("s_acc", el.ComponentType(el.PrimitiveType.U32, (1,))),
]
M10QGroundSpeed = ty.Annotated[
    jax.Array,
    el.Component("ground_speed", el.ComponentType(el.PrimitiveType.U32, (1,))),
]
M10QHeadingMotion = ty.Annotated[
    jax.Array,
    el.Component("heading_motion", el.ComponentType(el.PrimitiveType.I32, (1,))),
]
M10QValidFlags = ty.Annotated[
    jax.Array,
    el.Component("valid_flags", el.ComponentType(el.PrimitiveType.U8, (1,))),
]


@dataclass
class SensorOutput(el.Archetype):
    mag: SensorMag = field(default_factory=lambda: jnp.zeros(3, dtype=jnp.float32))
    gyro: SensorGyro = field(default_factory=lambda: jnp.zeros(3, dtype=jnp.float32))
    accel: SensorAccel = field(default_factory=lambda: jnp.zeros(3, dtype=jnp.float32))


@dataclass
class M10QOutput(el.Archetype):
    lat: M10QLat = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.int32))
    lon: M10QLon = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.int32))
    alt_msl: M10QAltMsl = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.int32))
    alt_wgs84: M10QAltWgs84 = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.int32))
    vel_ned: M10QVelNed = field(default_factory=lambda: jnp.zeros(3, dtype=jnp.int32))
    fix_type: M10QFixType = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.uint8))
    satellites: M10QSatellites = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.uint8))
    h_acc: M10QHAcc = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.uint32))
    v_acc: M10QVAcc = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.uint32))
    s_acc: M10QSAcc = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.uint32))
    ground_speed: M10QGroundSpeed = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.uint32))
    heading_motion: M10QHeadingMotion = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.int32))
    valid_flags: M10QValidFlags = field(default_factory=lambda: jnp.zeros(1, dtype=jnp.uint8))

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
# "IMU"       -- gyro/accel/mag in f32 (same schema as serial-bridge writes)
# "M10Q"      -- GPS in UBX integer format (same schema as serial-bridge writes)
# "ardupilot" -- motor data the bridge writes back (MotorTelemetry)
imu = world.spawn([SensorOutput()], name="IMU")
m10q = world.spawn([M10QOutput()], name="M10Q")
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
_home_lat_rad = math.radians(config.home_lat)


def post_step(tick: int, ctx: el.StepContext):
    """Copy sensor data from drone -> IMU/M10Q, motor commands from ardupilot -> drone."""
    if _start_time[0] is None:
        _start_time[0] = time.time()

    t = tick * config.dt

    # --- IMU: drone (f64) -> IMU entity (f32) ---
    try:
        gyro = np.array(ctx.read_component("drone.sim_gyro"), dtype=np.float32)
        accel = np.array(ctx.read_component("drone.sim_accel"), dtype=np.float32)
        mag = np.array(ctx.read_component("drone.magnetometer"), dtype=np.float32)

        ctx.write_component("IMU.gyro", gyro)
        ctx.write_component("IMU.accel", accel)
        ctx.write_component("IMU.mag", mag)
    except RuntimeError:
        pass

    # --- GPS: drone (f64 truth) -> noise -> M10Q entity (UBX integers) ---
    # Noise is injected here using numpy PRNG (not JAX) so each fix gets
    # an independent random draw.
    try:
        gps_set_raw = np.array(ctx.read_component("drone.gps_set"))
        gps_set_val = int(gps_set_raw.flat[0])
    except RuntimeError:
        gps_set_val = 0

    if gps_set_val:
        try:
            lat_truth = float(np.array(ctx.read_component("drone.gps_lat")).flat[0])
            lon_truth = float(np.array(ctx.read_component("drone.gps_lon")).flat[0])
            alt_truth = float(np.array(ctx.read_component("drone.gps_alt_msl")).flat[0])
            vel_truth = np.array(ctx.read_component("drone.gps_vel_ned"), dtype=np.float64).flatten()

            # Per-fix Gaussian noise matching M10Q observed characteristics
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

            ctx.write_component("M10Q.lat", np.array([lat_e7], dtype=np.int32))
            ctx.write_component("M10Q.lon", np.array([lon_e7], dtype=np.int32))
            ctx.write_component("M10Q.alt_msl", np.array([alt_mm], dtype=np.int32))
            ctx.write_component("M10Q.alt_wgs84", np.array([alt_mm], dtype=np.int32))
            ctx.write_component("M10Q.vel_ned", np.array(vel_ned_mms, dtype=np.int32))
            ctx.write_component("M10Q.fix_type", np.array([3], dtype=np.uint8))
            ctx.write_component("M10Q.satellites", np.array([12], dtype=np.uint8))
            ctx.write_component("M10Q.h_acc", np.array([h_acc_mm], dtype=np.uint32))
            ctx.write_component("M10Q.v_acc", np.array([v_acc_mm], dtype=np.uint32))
            ctx.write_component("M10Q.s_acc", np.array([s_acc_mms], dtype=np.uint32))
            ctx.write_component("M10Q.ground_speed", np.array([gs_mms], dtype=np.uint32))
            ctx.write_component("M10Q.heading_motion", np.array([hdg_e5], dtype=np.int32))
            ctx.write_component("M10Q.valid_flags", np.array([0x37], dtype=np.uint8))
        except RuntimeError:
            pass

    # --- Motors: ardupilot -> drone ---
    try:
        motor_cmd_f32 = np.array(ctx.read_component("ardupilot.motor_command"))
        ctx.write_component("drone.motor_command", motor_cmd_f32.astype(np.float64))
    except RuntimeError:
        ctx.write_component("drone.motor_command", np.zeros(4, dtype=np.float64))

    try:
        motor_pwm_u16 = np.array(ctx.read_component("ardupilot.motor_pwm"))
        ctx.write_component("drone.motor_pwm", motor_pwm_u16.astype(np.float64))
    except RuntimeError:
        ctx.write_component("drone.motor_pwm", np.zeros(4, dtype=np.float64))

    # Periodic status
    if t - _last_print[0] >= 1.0:
        _last_print[0] = t
        elapsed = time.time() - _start_time[0]
        rtf = t / elapsed if elapsed > 0 else 0
        try:
            pos = np.array(ctx.read_component("drone.world_pos"))
            z = pos[6] if len(pos) > 6 else 0.0
            cmd = np.array(ctx.read_component("drone.motor_command"))
            gps_info = ""
            try:
                glat = float(np.array(ctx.read_component("drone.gps_lat")).flat[0])
                glon = float(np.array(ctx.read_component("drone.gps_lon")).flat[0])
                gps_info = f" | gps=({glat:.4f},{glon:.4f})"
            except RuntimeError:
                gps_info = " | gps=n/a"
            print(
                f"  t={t:5.1f}s | z={z:+.2f}m | "
                f"motors=[{cmd[0]:.3f},{cmd[1]:.3f},{cmd[2]:.3f},{cmd[3]:.3f}] | "
                f"{rtf:.1f}x RT{gps_info}"
            )
        except Exception:
            print(f"  t={t:5.1f}s | (waiting for data)")


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
print('  Bridge reads: "IMU" entity (gyro/accel/mag)')
print('  Bridge reads: "M10Q" entity (GPS UBX integers)')
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
