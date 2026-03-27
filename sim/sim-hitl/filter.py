"""Discrete-time filters from the Elodin drone example."""

import math

import jax
from jax import numpy as jnp


class BiquadLPF:
    """Discrete-time second-order recursive linear filter."""

    def __init__(self, cutoff_freq: float, sample_freq: float):
        Q = 1 / math.sqrt(2)
        omega = 2 * math.pi * cutoff_freq / sample_freq
        alpha = math.sin(omega) / (2 * Q)
        a0 = 1 + alpha

        b0 = (1 - math.cos(omega)) / 2
        b1 = 1 - math.cos(omega)
        b2 = b0
        a1 = -2 * math.cos(omega)
        a2 = 1 - alpha

        b0 /= a0
        b1 /= a0
        b2 /= a0
        a1 /= a0
        a2 /= a0

        self.coefs = jnp.array([b0, b1, b2, a1, a2])

    def apply(self, delay: jax.Array, x_n: jax.Array) -> jax.Array:
        """Apply filter. delay is [x_n-1, x_n-2, y_n-1, y_n-2]. Returns updated delay."""
        assert delay.shape == (4, *x_n.shape)
        b0, b1, b2, a1, a2 = self.coefs
        x_n1, x_n2, y_n1, y_n2 = delay
        y_n = b0 * x_n + b1 * x_n1 + b2 * x_n2 - a1 * y_n1 - a2 * y_n2
        return jnp.array([x_n, x_n1, y_n, y_n1])
