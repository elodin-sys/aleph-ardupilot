"""6-DOF quadcopter physics for ArduPilot sim-HITL.

Modeled after the Elodin drone example (examples/drone/sim.py).
ArduPilot handles attitude control and motor mixing; this module provides
the physics plant: motor dynamics, thrust/torque computation, aerodynamic
drag, gravity, and ground constraint.

Motor commands arrive from the ardupilot-bridge as normalized [0,1] values
via the ``motor_command`` external_control component.
"""

import typing as ty
from dataclasses import dataclass, field

import elodin as el
import jax
import jax.numpy as jnp
import numpy as np
import sensors
from config import Config, MOT_PWM_THST_MAX, MOT_PWM_THST_MIN, MOT_TIME_CONST

# ---------------------------------------------------------------------------
# Component types
# ---------------------------------------------------------------------------

MotorCommand = ty.Annotated[
    jax.Array,
    el.Component(
        "motor_command",
        el.ComponentType(el.PrimitiveType.F64, (4,)),
        metadata={
            "element_names": "m0,m1,m2,m3",
            "priority": 100,
            "external_control": "true",
        },
    ),
]

MotorPwm = ty.Annotated[
    jax.Array,
    el.Component(
        "motor_pwm",
        el.ComponentType(el.PrimitiveType.F64, (4,)),
        metadata={"element_names": "m0,m1,m2,m3"},
    ),
]

MotorRpm = ty.Annotated[
    jax.Array,
    el.Component(
        "motor_rpm",
        el.ComponentType(el.PrimitiveType.F64, (4,)),
        metadata={"element_names": "m0,m1,m2,m3"},
    ),
]

Thrust = ty.Annotated[
    jax.Array,
    el.Component(
        "thrust",
        el.ComponentType(el.PrimitiveType.F64, (4,)),
        metadata={"priority": 98},
    ),
]

Torque = ty.Annotated[
    jax.Array,
    el.Component(
        "torque",
        el.ComponentType(el.PrimitiveType.F64, (4,)),
        metadata={"priority": 97},
    ),
]

BodyThrust = ty.Annotated[
    el.SpatialForce,
    el.Component(
        "body_thrust",
        metadata={"priority": 200, "element_names": "tx,ty,tz,x,y,z"},
    ),
]

BodyDrag = ty.Annotated[
    jax.Array,
    el.Component(
        "body_drag",
        el.ComponentType(el.PrimitiveType.F64, (3,)),
        metadata={"element_names": "x,y,z"},
    ),
]

# ---------------------------------------------------------------------------
# Archetypes
# ---------------------------------------------------------------------------


@dataclass
class Drone(el.Archetype):
    motor_command: MotorCommand = field(default_factory=lambda: jnp.zeros(4))
    motor_pwm: MotorPwm = field(default_factory=lambda: jnp.zeros(4))
    motor_rpm: MotorRpm = field(default_factory=lambda: jnp.zeros(4))
    thrust: Thrust = field(default_factory=lambda: jnp.zeros(4))
    torque: Torque = field(default_factory=lambda: jnp.zeros(4))
    body_thrust: BodyThrust = field(default_factory=lambda: el.SpatialForce())
    body_drag: BodyDrag = field(default_factory=lambda: jnp.zeros(3))


# ---------------------------------------------------------------------------
# Systems
# ---------------------------------------------------------------------------


@el.map
def motor_cmd_to_pwm(cmd: MotorCommand) -> MotorPwm:
    """Map normalized [0,1] motor commands from ArduPilot to PWM microseconds.

    The bridge normalises ArduPilot's raw PWM (1000-2000 us) to [0,1].
    We reconstruct the PWM value and clamp to the active thrust range.
    """
    pwm_raw = cmd * 1000.0 + 1000.0
    return jnp.clip(pwm_raw, MOT_PWM_THST_MIN, MOT_PWM_THST_MAX)


@el.map
def motor_thrust_response(
    pwm: MotorPwm, prev_thrust: Thrust, prev_torque: Torque, prev_rpm: MotorRpm
) -> tuple[Thrust, Torque, MotorRpm]:
    """RPM-based motor dynamics from the thrust curve CSV."""
    dt = Config.GLOBAL.fast_loop_time_step
    pwm_ref, thrust_ref, torque_ref, rpm_ref = Config.GLOBAL.thrust_curve()
    _, _, yaw_factor, _ = (
        np.array([np.zeros(4), np.zeros(4), Config.GLOBAL.frame.yaw_factor, np.ones(4)])
    )
    thrust_constant = np.linalg.lstsq(
        rpm_ref[:, np.newaxis] ** 2, thrust_ref, rcond=None
    )[0][0]
    torque_constant = np.linalg.lstsq(
        rpm_ref[:, np.newaxis] ** 2, torque_ref, rcond=None
    )[0][0]

    alpha = dt / (dt + MOT_TIME_CONST)
    rpm = jnp.interp(pwm, pwm_ref, rpm_ref)
    rpm = prev_rpm + alpha * (rpm - prev_rpm)

    thrust = rpm**2 * thrust_constant
    torque = rpm**2 * torque_constant * yaw_factor
    return thrust, torque, rpm


@el.map
def body_thrust_system(thrust: Thrust, torque: Torque) -> BodyThrust:
    """Combine per-motor thrust/torque into a single body-frame spatial force."""
    config = Config.GLOBAL
    thrust_dir = config.motor_thrust_directions
    torque_dir = config.motor_torque_axes

    linear = el.SpatialForce(linear=jnp.sum(thrust_dir * thrust[:, None], axis=0))
    yaw = el.SpatialForce(torque=jnp.sum(thrust_dir * torque[:, None], axis=0))
    pitch_roll = el.SpatialForce(torque=jnp.sum(torque_dir * thrust[:, None], axis=0))
    return linear + yaw + pitch_roll


@el.map
def drag(v: el.WorldVel) -> BodyDrag:
    rel_v = -v.linear()
    rel_v_norm = jnp.linalg.norm(rel_v)
    return 0.2 * 0.5 * rel_v * rel_v_norm


@el.map
def apply_body_forces(
    thrust: BodyThrust, drag_f: BodyDrag, pos: el.WorldPos, f: el.Force
) -> el.Force:
    return f + el.SpatialForce(linear=drag_f) + pos.angular() @ thrust


@el.map
def gravity(inertia: el.Inertia, f: el.Force) -> el.Force:
    return f + el.SpatialForce(linear=jnp.array([0.0, 0.0, -9.81]) * inertia.mass())


@el.map
def ground_constraint(pos: el.WorldPos, vel: el.WorldVel) -> tuple[el.WorldPos, el.WorldVel]:
    """Prevent the drone from falling through the ground."""
    p = pos.linear()
    v = vel.linear()
    omega = vel.angular()

    below = p[2] < 0.0
    new_z = jnp.where(below, 0.0, p[2])
    new_vz = jnp.where(below & (v[2] < 0), 0.0, v[2])

    damping = jnp.clip((0.5 - p[2]) / 0.5, 0.0, 1.0) * 0.95
    new_omega = omega * (1.0 - damping)

    new_pos = el.SpatialTransform(
        linear=jnp.array([p[0], p[1], new_z]),
        angular=pos.angular(),
    )
    new_vel = el.SpatialMotion(
        linear=jnp.array([v[0], v[1], new_vz]),
        angular=new_omega,
    )
    return new_pos, new_vel


# ---------------------------------------------------------------------------
# System composition
# ---------------------------------------------------------------------------


def inner_loop(run_count: int, system: el.System) -> el.System:
    out = system
    for _ in range(run_count - 1):
        out = out | system
    return out


def system() -> el.System:
    effectors = gravity | drag | motor_thrust_response | body_thrust_system | apply_body_forces

    inner_run_count = round(Config.GLOBAL.dt / Config.GLOBAL.fast_loop_time_step)

    inner = inner_loop(
        inner_run_count,
        el.six_dof(
            Config.GLOBAL.fast_loop_time_step,
            effectors,
            integrator=el.Integrator.SemiImplicit,
        )
        | sensors.imu,
    )

    return motor_cmd_to_pwm | inner | ground_constraint
