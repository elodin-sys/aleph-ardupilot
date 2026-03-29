"""Drone configuration for ArduPilot sim-HITL simulation.

Modeled after the Elodin drone example (examples/drone/config.py).
Physical parameters match the EDU-450 quadcopter. ArduPilot handles
attitude control and motor mixing; this config only covers the physics
plant and sensor models.
"""

import os
import typing as ty
from dataclasses import dataclass
from enum import Enum

import elodin as el
import numpy as np
from numpy._typing import NDArray


class Frame(Enum):
    QUAD_X = 0

    @property
    def yaw_factor(self) -> NDArray[np.float64]:
        if self == Frame.QUAD_X:
            return np.array([-1.0, -1.0, 1.0, 1.0])
        raise ValueError(f"Unsupported frame: {self}")


def motor_positions(angles: NDArray[np.float64], distance: float) -> NDArray[np.float64]:
    import jax.numpy as jnp

    x = jnp.sin(angles)
    y = -jnp.cos(angles)
    z = jnp.zeros_like(angles)
    return np.array(np.stack([x, y, z], axis=-1) * distance)


class _classproperty:
    def __init__(self, func):
        self.fget = func

    def __get__(self, obj, cls=None):
        if cls is None:
            cls = type(obj)
        return self.fget(cls)


# ArduPilot / motor parameters (from elodin drone example params.py)
MOT_SPIN_ARM = 0.10
MOT_SPIN_MIN = 0.12
MOT_SPIN_MAX = 0.95
MOT_PWM_MIN = 1050
MOT_PWM_MAX = 1900
MOT_TIME_CONST = 0.1  # seconds

INS_GYRO_FILTER = 40  # Hz
INS_ACCEL_FILTER = 20  # Hz

MOT_PWM_THST_MIN = MOT_PWM_MIN + (MOT_PWM_MAX - MOT_PWM_MIN) * MOT_SPIN_MIN
MOT_PWM_THST_MAX = MOT_PWM_MIN + (MOT_PWM_MAX - MOT_PWM_MIN) * MOT_SPIN_MAX


@dataclass
class Config:
    _GLOBAL: ty.ClassVar[ty.Optional[ty.Self]] = None

    mass: float
    inertia_diagonal: NDArray[np.float64]
    start_pos: NDArray[np.float64]
    motor_positions: NDArray[np.float64]
    motor_thrust_directions: NDArray[np.float64]
    motor_thrust_curve_path: str
    frame: Frame
    simulation_rate: float
    fast_loop_time_step: float
    simulation_time: float
    sensor_noise: bool

    # GPS -- matches AP_HOME so the bridge's NED conversions are consistent.
    home_lat: float = 37.7749
    home_lon: float = -122.4194
    home_alt: float = 10.0
    gps_rate: float = 5.0
    gps_hacc_std: float = 2.0       # HW M10Q observed ~2.0 m lat/lon std
    gps_vacc_std: float = 3.0       # HW M10Q observed ~5 m alt std (multipath)
    gps_sacc_std: float = 0.065     # HW M10Q observed ~62 mm/s vel_ned std
    gps_boot_delay: float = 2.0     # seconds before first fix
    gps_warmup_samples: int = 10    # noise-free fixes after boot

    @property
    def dt(self) -> float:
        return 1.0 / self.simulation_rate

    @property
    def total_sim_ticks(self) -> int:
        return int(self.simulation_time / self.dt)

    @property
    def spatial_transform(self) -> el.SpatialTransform:
        return el.SpatialTransform(
            linear=self.start_pos,
            angular=el.Quaternion.identity(),
        )

    @property
    def spatial_inertia(self) -> el.SpatialInertia:
        return el.SpatialInertia(
            mass=self.mass,
            inertia=self.inertia_diagonal,
        )

    @property
    def motor_torque_axes(self) -> NDArray[np.float64]:
        return np.cross(self.motor_positions, self.motor_thrust_directions)

    def thrust_curve(self) -> np.ndarray:
        path = os.path.join(os.path.dirname(__file__), self.motor_thrust_curve_path)
        return np.genfromtxt(path, delimiter=",", skip_header=1).transpose()

    @_classproperty
    def GLOBAL(cls) -> ty.Self:
        if cls._GLOBAL is None:
            raise ValueError("No global config set. Call set_as_global() first.")
        return cls._GLOBAL

    def set_as_global(self):
        Config._GLOBAL = self


# EDU-450 quadcopter -- Quad-X motor layout matching ArduPilot motor order:
#   Motor 0: Front Right (CW)
#   Motor 1: Rear Left (CW)
#   Motor 2: Front Left (CCW)
#   Motor 3: Rear Right (CCW)
#
#  (CCW) 2   0 (CW)
#         \ /
#          X
#         / \
#   (CW) 1   3 (CCW)
DEFAULT_CONFIG = Config(
    mass=1.0,
    inertia_diagonal=np.array([0.1, 0.1, 0.2]),
    start_pos=np.array([0.0, 0.0, 0.1]),
    motor_positions=motor_positions(
        np.pi * np.array([0.25, -0.75, 0.75, -0.25]), 0.24
    ),
    motor_thrust_directions=np.array([
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
    ]),
    motor_thrust_curve_path="./motor_thrust_curve.csv",
    simulation_rate=450.0,
    fast_loop_time_step=1.0 / 900.0,
    simulation_time=60.0,
    sensor_noise=True,
    frame=Frame.QUAD_X,
)
