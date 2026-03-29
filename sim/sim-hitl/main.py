"""ArduPilot sim-HITL simulation using Elodin physics.

The ardupilot-bridge (running on the Aleph) connects to this simulation's
Elodin-DB.  Data flows entirely through the DB -- the same code path as
real hardware:

    Simulation  -->  "IMU" entity (gyro/accel/mag)  -->  bridge subscribes
    bridge writes "ardupilot" entity (motor_command)  -->  Simulation reads

Usage:
    elodin editor sim/sim-hitl/main.py   # with 3D visualisation
    elodin run sim/sim-hitl/main.py      # headless
"""

import time

import elodin as el
import jax.numpy as jnp
import numpy as np

from config import DEFAULT_CONFIG, Config
from sensors import IMU
from sim import Drone, system

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

config = DEFAULT_CONFIG
config.set_as_global()

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
    ],
    name="drone",
)

# Interface entities matching the bridge's VTable namespaces.
# "IMU"       -- sensor data the bridge subscribes to (matches serial-bridge)
# "ardupilot" -- motor data the bridge writes (same as MotorTelemetry)
# Spawned empty; the post_step populates them with f32 data matching the
# bridge's exact schema.  This avoids f32/f64 dtype conflicts with the
# physics systems which use f64 internally.
imu = world.spawn([], name="IMU")
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
            graph "drone.gyro" name="Gyroscope f64 (physics)"
            graph "IMU.gyro" name="Gyroscope f32 (to bridge)"
            graph "drone.accel" name="Accelerometer (physics)"
            graph "drone.magnetometer" name="Magnetometer (physics)"
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


def post_step(tick: int, ctx: el.StepContext):
    """Copy sensor data from drone -> IMU, motor commands from ardupilot -> drone."""
    if _start_time[0] is None:
        _start_time[0] = time.time()

    t = tick * config.dt

    # --- Sensors: drone (f64) -> IMU (f32) ---
    try:
        gyro = np.array(ctx.read_component("drone.gyro"), dtype=np.float32)
        accel = np.array(ctx.read_component("drone.accel"), dtype=np.float32)
        mag = np.array(ctx.read_component("drone.magnetometer"), dtype=np.float32)

        ctx.write_component("IMU.gyro", gyro)
        ctx.write_component("IMU.accel", accel)
        ctx.write_component("IMU.mag", mag)
    except RuntimeError:
        pass  # first few ticks may not have sensor data yet

    # --- Motors: ardupilot -> drone ---
    # Default to zeros until the bridge connects and ArduPilot sends servo output.
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
            print(
                f"  t={t:5.1f}s | z={z:+.2f}m | "
                f"motors=[{cmd[0]:.3f},{cmd[1]:.3f},{cmd[2]:.3f},{cmd[3]:.3f}] | "
                f"{rtf:.1f}x RT"
            )
        except Exception:
            print(f"  t={t:5.1f}s | (waiting for data)")


# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------

sim_system = system()

print("ArduPilot sim-HITL Simulation")
print("=============================")
print(f"Mass: {config.mass:.1f} kg")
print(f"Simulation rate: {config.simulation_rate:.0f} Hz (inner: {1/config.fast_loop_time_step:.0f} Hz)")
print(f"Duration: {config.simulation_time:.0f} s")
print()
print("Data flows through the DB -- same code path as real hardware.")
print('  Bridge reads: "IMU" entity (gyro/accel/mag)')
print('  Bridge writes: "ardupilot" entity (motor_command/motor_pwm)')
print()
print("Deploy sim-hitl config to the Aleph:")
print("  ./deploy.sh -c sim-hitl -h <aleph-ip> -u root -i ssh/aleph-key")
print()

world.run(
    sim_system,
    simulation_rate=config.simulation_rate,
    generate_real_time=True,
    post_step=post_step,
)
