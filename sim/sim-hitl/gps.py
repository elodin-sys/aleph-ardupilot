"""GPS sensor simulation for sim-HITL.

Computes truth-state GPS measurements (LLA position, NED velocity) from the
drone's ENU physics state.  The Elodin system graph handles only the
coordinate conversion and timer gating -- per-fix Gaussian noise is injected
in ``post_step`` (main.py) using numpy PRNG before writing to the M10Q
interface entity.  This avoids JAX JIT issues with u64 PRNG keys.

The post_step converts noisy f64 values to UBX integer format and writes them
to the ``M10Q`` DB entity, matching the exact schema the ardupilot-bridge
expects.  The bridge is completely unaware a simulation is involved.

Coordinate math uses the flat-earth approximation (accurate within ~10 km).
"""

import typing as ty
from dataclasses import dataclass, field

import elodin as el
import jax
import jax.numpy as jnp
from config import Config

WGS84_A = 6378137.0  # semi-major axis (meters)

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


@el.map_seq
def gps_model(
    gps_set: GpsSet,
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
]:
    """Produce a truth GPS measurement from the physics state.

    When ``gps_set == 1`` (timer fired this tick):
      1. Read ENU position / velocity from the physics engine.
      2. Convert ENU position to LLA via flat-earth approximation.
      3. Convert ENU velocity to NED.
      4. Compute ground speed and heading of motion.
    When ``gps_set == 0``: hold previous values (zero computation).

    Uses @el.map_seq so jax.lax.cond truly skips the conversion on
    non-fire ticks (~755/760 per second). Plain arrays are extracted
    from spatial types before the cond to avoid IREE closure issues.
    """
    pos_enu = p.linear()
    vel_enu = v.linear()

    def _compute_fix(operand):
        pe, ve = operand
        cfg = Config.GLOBAL
        home_lat_rad = jnp.deg2rad(cfg.home_lat)
        vel_ned = jnp.array([ve[1], ve[0], -ve[2]])
        lat = cfg.home_lat + (pe[1] / WGS84_A) * (180.0 / jnp.pi)
        lon = cfg.home_lon + (
            pe[0] / (WGS84_A * jnp.cos(home_lat_rad))
        ) * (180.0 / jnp.pi)
        alt = cfg.home_alt + pe[2]
        gs = jnp.sqrt(vel_ned[0] ** 2 + vel_ned[1] ** 2)
        hdg = jnp.rad2deg(jnp.arctan2(vel_ned[1], vel_ned[0]))
        hdg = jnp.where(hdg < 0.0, hdg + 360.0, hdg)
        return lat, lon, alt, vel_ned, gs, hdg

    def _hold_prev(operand):
        return prev_lat, prev_lon, prev_alt, prev_vel, prev_gs, prev_hdg

    return jax.lax.cond(
        gps_set == 1, _compute_fix, _hold_prev, (pos_enu, vel_enu)
    )


# Composed GPS system: timer -> model
gps = advance_gps_timer | gps_model
