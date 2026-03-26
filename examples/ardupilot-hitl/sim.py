"""6-DOF quadcopter physics simulation for ArduPilot HITL.

Adapted from the Elodin Betaflight SITL example. Implements rigid body
dynamics with motor thrust, aerodynamic drag, and ground constraint.
"""

import numpy as np
from config import DroneConfig


class QuadSim:
    """Simple 6-DOF quadcopter simulator."""

    def __init__(self, config: DroneConfig):
        self.config = config
        self.dt = config.dt
        self.g = 9.81

        # State (ENU world frame)
        self.position = np.array([0.0, 0.0, 0.0])  # meters
        self.velocity = np.array([0.0, 0.0, 0.0])  # m/s
        self.quaternion = np.array([0.0, 0.0, 0.0, 1.0])  # [x, y, z, w]
        self.angular_velocity = np.array([0.0, 0.0, 0.0])  # rad/s (body frame)

        # Motor state
        self.motor_thrust = np.zeros(4)  # current thrust per motor (N)
        self.motor_command = np.zeros(4)  # commanded throttle [0, 1]

        # Inertia tensor (simplified diagonal)
        ixx = config.mass * config.arm_length**2 / 2
        izz = config.mass * config.arm_length**2
        self.inertia = np.diag([ixx, ixx, izz])
        self.inertia_inv = np.linalg.inv(self.inertia)

    def step(self, motor_commands: np.ndarray):
        """Advance one physics step with given motor commands [0, 1]."""
        self.motor_command = np.clip(motor_commands, 0.0, 1.0)

        # Motor dynamics (first-order low-pass)
        alpha = self.dt / (self.dt + self.config.motor_time_constant)
        target_thrust = self.motor_command * self.config.motor_max_thrust
        self.motor_thrust += alpha * (target_thrust - self.motor_thrust)

        # Body forces and torques
        force_body = np.array([0.0, 0.0, 0.0])
        torque_body = np.array([0.0, 0.0, 0.0])

        for i in range(4):
            pos = np.array(self.config.motor_positions[i])
            thrust = self.motor_thrust[i]
            direction = self.config.motor_directions[i]

            force_body[2] += thrust  # thrust along body Z (up in FLU)
            torque_body += np.cross(pos, np.array([0, 0, thrust]))
            torque_body[2] += direction * thrust * 0.01  # yaw reaction torque

        # Rotate body force to world frame
        rot = self._quat_to_matrix(self.quaternion)
        force_world = rot @ force_body

        # Gravity (world frame, ENU: Z is up)
        force_world[2] -= self.config.mass * self.g

        # Linear drag
        force_world -= self.config.drag_coeff * np.linalg.norm(self.velocity) * self.velocity

        # Angular drag
        torque_body -= self.config.angular_drag_coeff * self.angular_velocity

        # Semi-implicit integration
        accel = force_world / self.config.mass
        self.velocity += accel * self.dt
        self.position += self.velocity * self.dt

        angular_accel = self.inertia_inv @ (
            torque_body - np.cross(self.angular_velocity, self.inertia @ self.angular_velocity)
        )
        self.angular_velocity += angular_accel * self.dt
        self.quaternion = self._integrate_quaternion(
            self.quaternion, self.angular_velocity, self.dt
        )

        # Ground constraint
        if self.position[2] < 0.0:
            self.position[2] = 0.0
            if self.velocity[2] < 0.0:
                self.velocity[2] = 0.0
            self.angular_velocity *= 0.95  # ground friction

    def get_accel_body(self) -> np.ndarray:
        """Specific force in body frame (what an accelerometer reads)."""
        rot = self._quat_to_matrix(self.quaternion)
        gravity_world = np.array([0.0, 0.0, -self.g])
        gravity_body = rot.T @ gravity_world
        accel_world = (self.velocity - self._prev_velocity) / self.dt if hasattr(self, '_prev_velocity') else np.zeros(3)
        accel_body = rot.T @ accel_world
        return accel_body - gravity_body

    def get_pressure(self) -> float:
        """Barometric pressure from altitude."""
        return 101325.0 - 12.0 * self.position[2]

    @staticmethod
    def _quat_to_matrix(q):
        x, y, z, w = q
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)],
        ])

    @staticmethod
    def _integrate_quaternion(q, omega, dt):
        ox, oy, oz = omega * dt * 0.5
        dq = np.array([
            q[3]*ox + q[1]*oz - q[2]*oy,
            q[3]*oy + q[2]*ox - q[0]*oz,
            q[3]*oz + q[0]*oy - q[1]*ox,
            -q[0]*ox - q[1]*oy - q[2]*oz,
        ])
        q_new = q + dq
        return q_new / np.linalg.norm(q_new)
