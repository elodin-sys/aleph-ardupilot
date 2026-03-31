"""IMU sensor simulation with noise and filtering.

Modeled after the Elodin drone example (examples/drone/sensors.py).
Produces gyro, accel, and magnetometer readings in body frame with
configurable Gaussian noise, bias drift, and biquad low-pass filtering.
"""

import typing as ty
from dataclasses import dataclass, field

import elodin as el
import filter as flt
import jax
import jax.numpy as jnp
import jax.random as rng
from config import Config, INS_ACCEL_FILTER, INS_GYRO_FILTER

SensorTick = ty.Annotated[jax.Array, el.Component("sensor_tick", el.ComponentType.U64)]

Gyro = ty.Annotated[
    jax.Array,
    el.Component(
        "sim_gyro",
        el.ComponentType(el.PrimitiveType.F64, (3,)),
        metadata={"priority": 90, "element_names": "x,y,z"},
    ),
]
GyroBias = ty.Annotated[
    jax.Array,
    el.Component("gyro_bias", el.ComponentType(el.PrimitiveType.F64, (3,))),
]
GyroLPFDelay = ty.Annotated[
    jax.Array,
    el.Component("gyro_lpf_delay", el.ComponentType(el.PrimitiveType.F64, (4, 3))),
]

Accel = ty.Annotated[
    jax.Array,
    el.Component(
        "sim_accel",
        el.ComponentType(el.PrimitiveType.F64, (3,)),
        metadata={"priority": 89, "element_names": "x,y,z"},
    ),
]
AccelBias = ty.Annotated[
    jax.Array,
    el.Component("accel_bias", el.ComponentType(el.PrimitiveType.F64, (3,))),
]
AccelLPFDelay = ty.Annotated[
    jax.Array,
    el.Component("accel_lpf_delay", el.ComponentType(el.PrimitiveType.F64, (4, 3))),
]

Magnetometer = ty.Annotated[
    jax.Array,
    el.Component(
        "magnetometer",
        el.ComponentType(el.PrimitiveType.F64, (3,)),
        metadata={"priority": 87, "element_names": "x,y,z"},
    ),
]
MagnetometerBias = ty.Annotated[
    jax.Array,
    el.Component("magnetometer_bias", el.ComponentType(el.PrimitiveType.F64, (3,))),
]


class Noise:
    def __init__(self, seed: int, device: int, noise_cov: float, bias_drift_cov: float):
        self.noise_cov = noise_cov
        self.bias_drift_cov = bias_drift_cov
        self.key = rng.fold_in(rng.key(seed), device)

    def drift_bias(self, bias: jax.Array, tick: SensorTick, dt: float) -> jax.Array:
        key = rng.fold_in(self.key, tick)
        std = jnp.sqrt(self.bias_drift_cov)
        drift = std * rng.normal(key, shape=bias.shape, dtype=bias.dtype) * dt
        return bias + drift

    def sample(self, m: jax.Array, bias: jax.Array, tick: SensorTick) -> jax.Array:
        key = rng.fold_in(self.key, tick)
        std = jnp.sqrt(self.noise_cov)
        noise = std * rng.normal(key, shape=m.shape, dtype=m.dtype)
        return m + noise + bias


gyro_noise = Noise(0, 0, 0.001, 0.001)
init_gyro_bias = jnp.array([0.0025, 0.0001, 0.0005])
accel_noise = Noise(0, 1, 0.001, 0.0)
mag_noise = Noise(0, 2, 6.0, 0.0)

# IGRF Earth magnetic field at San Francisco (37.77N, 122.42W) in ENU,
# scaled to match the BMM350 firmware output units observed in real-world
# test-db-csv (magnitude ~247).  Raw IGRF: ~48.4 uT total, dec ~13 deg E,
# inc ~61 deg.  Scale factor ~5.1 converts uT to firmware units.
MAG_FIELD_ENU = jnp.array([27.0, 116.0, -216.0])


@dataclass
class IMU(el.Archetype):
    sensor_tick: SensorTick = field(default_factory=lambda: jnp.array(0))
    gyro: Gyro = field(default_factory=lambda: jnp.zeros(3))
    gyro_bias: GyroBias = field(default_factory=lambda: jnp.array(init_gyro_bias))
    accel: Accel = field(default_factory=lambda: jnp.zeros(3))
    accel_bias: AccelBias = field(default_factory=lambda: jnp.zeros(3))
    mag: Magnetometer = field(default_factory=lambda: jnp.array(MAG_FIELD_ENU))
    mag_bias: MagnetometerBias = field(default_factory=lambda: jnp.zeros(3))
    gyro_lpf_delay: GyroLPFDelay = field(default_factory=lambda: jnp.zeros((4, 3)))
    accel_lpf_delay: AccelLPFDelay = field(default_factory=lambda: jnp.zeros((4, 3)))


@el.map
def advance_sensor_tick(tick: SensorTick) -> SensorTick:
    return tick + 1


@el.map
def update_gyro_noise(tick: SensorTick, bias: GyroBias) -> GyroBias:
    dt = Config.GLOBAL.dt
    return gyro_noise.drift_bias(bias, tick, dt)


@el.map
def gyro_system(
    tick: SensorTick,
    p: el.WorldPos,
    v: el.WorldVel,
    delay: GyroLPFDelay,
    bias: GyroBias,
) -> tuple[GyroLPFDelay, Gyro]:
    dt = Config.GLOBAL.dt
    body_v = p.angular().inverse() @ v.angular()
    if Config.GLOBAL.sensor_noise:
        body_v = gyro_noise.sample(body_v, bias, tick)
    lpf = flt.BiquadLPF(INS_GYRO_FILTER, 1.0 / dt)
    new_delay = lpf.apply(delay, body_v)
    return (new_delay, new_delay[2])


@el.map
def accel_system(
    tick: SensorTick,
    p: el.WorldPos,
    a: el.WorldAccel,
    delay: AccelLPFDelay,
    bias: AccelBias,
) -> tuple[AccelLPFDelay, Accel]:
    dt = Config.GLOBAL.dt
    body_a = p.angular().inverse() @ (a.linear() + jnp.array([0, 0, 9.81]))

    # The ground constraint clamps position/velocity but doesn't generate a
    # normal force, so WorldAccel on the ground is just gravity and the +9.81
    # correction cancels it to ~0.  A real accelerometer reads the support
    # force, which is +9.81 m/s^2 upward in body frame for a stationary drone.
    z = p.linear()[2]
    on_ground = z < 0.02
    gravity_body = p.angular().inverse() @ jnp.array([0.0, 0.0, 9.81])
    body_a = jnp.where(on_ground, gravity_body, body_a)

    if Config.GLOBAL.sensor_noise:
        body_a = accel_noise.sample(body_a, bias, tick)
    lpf = flt.BiquadLPF(INS_ACCEL_FILTER, 1.0 / dt)
    new_delay = lpf.apply(delay, body_a)
    return (new_delay, new_delay[2])


@el.map
def mag_system(
    tick: SensorTick,
    p: el.WorldPos,
    bias: MagnetometerBias,
    prev_mag: Magnetometer,
) -> Magnetometer:
    dt = Config.GLOBAL.dt
    data_rate = 1.0 / 100.0
    tick_rate = round(data_rate / dt)
    body_mag_ref = p.angular().inverse() @ MAG_FIELD_ENU
    if Config.GLOBAL.sensor_noise:
        body_mag_ref = mag_noise.sample(body_mag_ref, bias, tick)
    return jax.lax.cond(
        tick % tick_rate == 0,
        lambda _: body_mag_ref,
        lambda _: prev_mag,
        None,
    )


imu = advance_sensor_tick | update_gyro_noise | gyro_system | accel_system | mag_system
