"""ArduPilot HITL simulation using Elodin physics.

This example connects to the ardupilot-bridge's HITL socket and runs a
6-DOF quadcopter simulation, sending sensor data and receiving motor
commands in lockstep -- matching the Betaflight SITL example pattern.

Usage:
    # With bridge running on Aleph (or locally):
    python3 sim/ardupilot-hitl/main.py

    # Specify bridge address:
    python3 sim/ardupilot-hitl/main.py --host 10.101.100.71 --port 9100
"""

import argparse
import socket
import struct
import time
import numpy as np

from config import DroneConfig
from sim import QuadSim

FDM_PACKET_FORMAT = "<18d"  # 18 doubles, little-endian
FDM_PACKET_SIZE = struct.calcsize(FDM_PACKET_FORMAT)
MOTOR_PACKET_FORMAT = "<4d"  # 4 doubles, little-endian
MOTOR_PACKET_SIZE = struct.calcsize(MOTOR_PACKET_FORMAT)


def build_fdm_packet(sim: QuadSim, timestamp: float) -> bytes:
    """Build an FDM sensor packet from simulation state."""
    gyro = sim.angular_velocity
    accel = sim.get_accel_body()
    quat = sim.quaternion  # [x, y, z, w]
    vel = sim.velocity
    pos = sim.position
    pressure = sim.get_pressure()

    return struct.pack(
        FDM_PACKET_FORMAT,
        timestamp,
        gyro[0], gyro[1], gyro[2],
        accel[0], accel[1], accel[2],
        quat[3], quat[0], quat[1], quat[2],  # wxyz order
        vel[0], vel[1], vel[2],
        pos[0], pos[1], pos[2],
        pressure,
    )


def parse_motor_packet(data: bytes) -> np.ndarray:
    """Parse motor command packet from bridge."""
    values = struct.unpack(MOTOR_PACKET_FORMAT, data[:MOTOR_PACKET_SIZE])
    return np.array(values)


def main():
    parser = argparse.ArgumentParser(description="ArduPilot HITL Simulation")
    parser.add_argument("--host", default="127.0.0.1", help="Bridge HITL host")
    parser.add_argument("--port", type=int, default=9100, help="Bridge HITL port")
    parser.add_argument("--duration", type=float, default=20.0, help="Simulation duration (s)")
    parser.add_argument("--rate", type=float, default=1000.0, help="Simulation rate (Hz)")
    args = parser.parse_args()

    config = DroneConfig(
        simulation_rate=args.rate,
        simulation_duration=args.duration,
        bridge_host=args.host,
        bridge_hitl_port=args.port,
    )

    sim = QuadSim(config)
    dt = config.dt
    total_ticks = int(config.simulation_duration * config.simulation_rate)

    print(f"Connecting to ardupilot-bridge HITL at {args.host}:{args.port}")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((args.host, args.port))
    sock.settimeout(1.0)
    print(f"Connected. Running {total_ticks} ticks at {config.simulation_rate} Hz")

    motors = np.zeros(4)
    start_time = time.time()

    try:
        for tick in range(total_ticks):
            timestamp = tick * dt

            sim._prev_velocity = sim.velocity.copy()
            sim.step(motors)

            fdm = build_fdm_packet(sim, timestamp)
            sock.sendall(fdm)

            try:
                data = b""
                while len(data) < MOTOR_PACKET_SIZE:
                    chunk = sock.recv(MOTOR_PACKET_SIZE - len(data))
                    if not chunk:
                        print("Bridge disconnected")
                        return
                    data += chunk
                motors = parse_motor_packet(data)
            except socket.timeout:
                pass

            if tick % 1000 == 0:
                elapsed = time.time() - start_time
                real_time_factor = timestamp / elapsed if elapsed > 0 else 0
                print(
                    f"t={timestamp:.1f}s pos=[{sim.position[0]:.2f}, "
                    f"{sim.position[1]:.2f}, {sim.position[2]:.2f}] "
                    f"motors=[{motors[0]:.2f}, {motors[1]:.2f}, "
                    f"{motors[2]:.2f}, {motors[3]:.2f}] "
                    f"RTF={real_time_factor:.1f}x"
                )

    except KeyboardInterrupt:
        print("\nSimulation interrupted")
    finally:
        sock.close()
        elapsed = time.time() - start_time
        print(f"Simulation complete: {total_ticks} ticks in {elapsed:.1f}s")


if __name__ == "__main__":
    main()
