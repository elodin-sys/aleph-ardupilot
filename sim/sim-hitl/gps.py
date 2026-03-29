"""GPS sensor simulation for sim-HITL.

Converts the drone's ENU truth state to simulated SAM-M10Q measurements
(LLA position, NED velocity) with configurable Gaussian noise.  GPS data
is stored as f64 Elodin components on the drone entity.  The ardupilot-
bridge subscribes directly to these components via ``GpsSimInput`` (defined
in ``elodin.rs``) and converts LLA to NED for the ArduPilot JSON packet.

In hardware mode the bridge reads from the ``M10Q`` vtable instead (UBX
integer format from the real receiver).  The bridge tries M10Q first and
falls back to the drone entity's GPS components automatically.

Coordinate math uses the flat-earth approximation (accurate within ~10 km).

Noise parameters are derived from the SAM-M10Q datasheet:
  CEP 1.5 m  ->  h_acc_std ~ 1.3 m, v_acc_std ~ 2.0 m
  Velocity accuracy 0.05 m/s
"""

import typing as ty
from dataclasses import dataclass, field

import elodin as el
import jax
import jax.numpy as jnp
import jax.random as rng
from config import Config

WGS84_A = 6378137.0  # semi-major axis (meters)

# Deterministic PRNG base key for GPS noise, distinct from IMU seeds (0-2).
_gps_key = rng.fold_in(rng.key(0), 10)

# ---------------------------------------------------------------------------
# Components
# ---------------------------------------------------------------------------

GpsTimer = ty.Annotated[
    jax.Array,
    el.Component("gps_timer", el.ComponentType.F64),
]

GpsElapsed = ty.Annotated[
    jax.Array,
    el.Component("gps_elapsed", el.ComponentType.F64),
]

GpsSet = ty.Annotated[
    jax.Array,
    el.Component("gps_set", el.ComponentType.U64),
]

GpsWarmupCount = ty.Annotated[
    jax.Array,
    el.Component("gps_warmup_count", el.ComponentType.U64),
]

GpsLat = ty.Annotated[
    jax.Array,
    el.Component(
        "gps_lat",
        el.ComponentType.F64,
        metadata={"priority": 80, "element_names": "deg"},
    ),
]

GpsLon = ty.Annotated[
    jax.Array,
    el.Component(
        "gps_lon",
        el.ComponentType.F64,
        metadata={"priority": 79, "element_names": "deg"},
    ),
]

GpsAltMsl = ty.Annotated[
    jax.Array,
    el.Component(
        "gps_alt_msl",
        el.ComponentType.F64,
        metadata={"priority": 78, "element_names": "m"},
    ),
]

GpsVelNed = ty.Annotated[
    jax.Array,
    el.Component(
        "gps_vel_ned",
        el.ComponentType(el.PrimitiveType.F64, (3,)),
        metadata={"priority": 77, "element_names": "n,e,d"},
    ),
]

GpsGroundSpeed = ty.Annotated[
    jax.Array,
    el.Component(
        "gps_ground_speed",
        el.ComponentType.F64,
        metadata={"priority": 76, "element_names": "m/s"},
    ),
]

GpsHeading = ty.Annotated[
    jax.Array,
    el.Component(
        "gps_heading",
        el.ComponentType.F64,
        metadata={"priority": 75, "element_names": "deg"},
    ),
]

# ---------------------------------------------------------------------------
# Archetype
# ---------------------------------------------------------------------------


@dataclass
class GPS(el.Archetype):
    gps_timer: GpsTimer = field(default_factory=lambda: jnp.float64(0.0))
    gps_elapsed: GpsElapsed = field(default_factory=lambda: jnp.float64(0.0))
    gps_set: GpsSet = field(default_factory=lambda: jnp.uint64(0))
    gps_warmup_count: GpsWarmupCount = field(default_factory=lambda: jnp.uint64(0))
    gps_lat: GpsLat = field(default_factory=lambda: jnp.float64(0.0))
    gps_lon: GpsLon = field(default_factory=lambda: jnp.float64(0.0))
    gps_alt_msl: GpsAltMsl = field(default_factory=lambda: jnp.float64(0.0))
    gps_vel_ned: GpsVelNed = field(default_factory=lambda: jnp.zeros(3))
    gps_ground_speed: GpsGroundSpeed = field(default_factory=lambda: jnp.float64(0.0))
    gps_heading: GpsHeading = field(default_factory=lambda: jnp.float64(0.0))


# ---------------------------------------------------------------------------
# Systems
# ---------------------------------------------------------------------------


@el.map
def advance_gps_timer(
    timer: GpsTimer,
    elapsed: GpsElapsed,
) -> tuple[GpsTimer, GpsElapsed, GpsSet]:
    """Increment the GPS timer and fire a pulse at the configured rate.

    The timer starts producing fixes only after ``gps_boot_delay`` has
    elapsed, simulating a cold-start acquisition period.
    """
    dt = Config.GLOBAL.dt
    gps_dt = 1.0 / Config.GLOBAL.gps_rate
    boot_delay = Config.GLOBAL.gps_boot_delay

    new_elapsed = elapsed + dt
    new_timer = timer + dt
    timer_fire = new_timer >= gps_dt
    new_timer = jnp.where(timer_fire, 0.0, new_timer)

    booted = new_elapsed >= boot_delay
    gps_fire = jnp.logical_and(timer_fire, booted)

    return (
        new_timer,
        new_elapsed,
        jnp.where(gps_fire, jnp.uint64(1), jnp.uint64(0)),
    )


@el.map
def gps_model(
    gps_set: GpsSet,
    warmup: GpsWarmupCount,
    p: el.WorldPos,
    v: el.WorldVel,
    prev_lat: GpsLat,
    prev_lon: GpsLon,
    prev_alt: GpsAltMsl,
    prev_vel: GpsVelNed,
    prev_gs: GpsGroundSpeed,
    prev_hdg: GpsHeading,
) -> tuple[
    GpsLat,
    GpsLon,
    GpsAltMsl,
    GpsVelNed,
    GpsGroundSpeed,
    GpsHeading,
    GpsWarmupCount,
]:
    """Produce a GPS measurement from the physics truth state.

    When ``gps_set == 1`` (timer fired this tick):
      1. Read ENU position / velocity from the physics engine.
      2. Convert ENU position to LLA via flat-earth approximation.
      3. Convert ENU velocity to NED.
      4. Inject Gaussian noise (suppressed during warmup).
      5. Compute ground speed and heading of motion.
    When ``gps_set == 0``: hold previous values.
    """
    cfg = Config.GLOBAL
    home_lat_rad = jnp.deg2rad(cfg.home_lat)

    # ----- truth state -----
    pos_enu = p.linear()  # [East, North, Up] metres
    vel_enu = v.linear()  # [East, North, Up] m/s

    # ENU velocity -> NED
    vel_ned = jnp.array([vel_enu[1], vel_enu[0], -vel_enu[2]])

    # ----- noise (zero during warmup) -----
    in_warmup = warmup < cfg.gps_warmup_samples
    noise_scale = jnp.where(in_warmup, 0.0, 1.0)

    if cfg.sensor_noise:
        key = rng.fold_in(_gps_key, warmup)
        kp, kv = rng.split(key)
        pos_noise = jnp.array([
            cfg.gps_hacc_std * rng.normal(rng.fold_in(kp, 0), shape=()),
            cfg.gps_hacc_std * rng.normal(rng.fold_in(kp, 1), shape=()),
            cfg.gps_vacc_std * rng.normal(rng.fold_in(kp, 2), shape=()),
        ])
        vel_noise = jnp.array([
            cfg.gps_sacc_std * rng.normal(rng.fold_in(kv, 0), shape=()),
            cfg.gps_sacc_std * rng.normal(rng.fold_in(kv, 1), shape=()),
            cfg.gps_sacc_std * rng.normal(rng.fold_in(kv, 2), shape=()),
        ])
    else:
        pos_noise = jnp.zeros(3)
        vel_noise = jnp.zeros(3)

    noisy_enu = pos_enu + noise_scale * pos_noise
    noisy_vel = vel_ned + noise_scale * vel_noise

    # ----- flat-earth ENU -> LLA -----
    lat = cfg.home_lat + (noisy_enu[1] / WGS84_A) * (180.0 / jnp.pi)
    lon = cfg.home_lon + (
        noisy_enu[0] / (WGS84_A * jnp.cos(home_lat_rad))
    ) * (180.0 / jnp.pi)
    alt = cfg.home_alt + noisy_enu[2]

    # ----- derived quantities -----
    gs = jnp.sqrt(noisy_vel[0] ** 2 + noisy_vel[1] ** 2)
    hdg = jnp.rad2deg(jnp.arctan2(noisy_vel[1], noisy_vel[0]))
    hdg = jnp.where(hdg < 0.0, hdg + 360.0, hdg)

    # ----- gate: only update on GPS fire -----
    fire = gps_set == 1
    new_lat = jnp.where(fire, lat, prev_lat)
    new_lon = jnp.where(fire, lon, prev_lon)
    new_alt = jnp.where(fire, alt, prev_alt)
    new_vel = jnp.where(fire, noisy_vel, prev_vel)
    new_gs = jnp.where(fire, gs, prev_gs)
    new_hdg = jnp.where(fire, hdg, prev_hdg)
    new_warmup = jnp.where(fire, warmup + jnp.uint64(1), warmup)

    return new_lat, new_lon, new_alt, new_vel, new_gs, new_hdg, new_warmup


# Composed GPS system: timer -> model
gps = advance_gps_timer | gps_model

