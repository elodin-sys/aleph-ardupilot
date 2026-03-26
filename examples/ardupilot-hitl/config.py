"""Drone configuration for ArduPilot HITL simulation."""

from dataclasses import dataclass, field
from typing import List, Tuple
import math


@dataclass
class DroneConfig:
    mass: float = 0.8  # kg
    arm_length: float = 0.12  # meters (center to motor)
    motor_max_thrust: float = 15.0  # Newtons per motor
    motor_time_constant: float = 0.02  # seconds (first-order lag)
    drag_coeff: float = 0.1  # linear drag coefficient
    angular_drag_coeff: float = 0.01  # angular drag coefficient

    simulation_rate: float = 1000.0  # Hz (physics tick rate)
    simulation_duration: float = 20.0  # seconds

    # ArduPilot bridge connection
    bridge_host: str = "127.0.0.1"
    bridge_hitl_port: int = 9100

    # Motor positions in body frame FLU [forward, left, up] (meters)
    # Quad-X layout matching ArduPilot motor order:
    #   Motor 1: Front Right (CW)
    #   Motor 2: Back Left (CW)
    #   Motor 3: Front Left (CCW)
    #   Motor 4: Back Right (CCW)
    @property
    def motor_positions(self) -> List[Tuple[float, float, float]]:
        d = self.arm_length / math.sqrt(2)
        return [
            (d, -d, 0.0),   # Motor 1: FR
            (-d, d, 0.0),   # Motor 2: BL
            (d, d, 0.0),    # Motor 3: FL
            (-d, -d, 0.0),  # Motor 4: BR
        ]

    @property
    def motor_directions(self) -> List[float]:
        """Yaw torque direction: +1 = CCW, -1 = CW."""
        return [-1.0, -1.0, 1.0, 1.0]

    @property
    def dt(self) -> float:
        return 1.0 / self.simulation_rate

    @property
    def hover_throttle(self) -> float:
        g = 9.81
        return (self.mass * g) / (4 * self.motor_max_thrust)
