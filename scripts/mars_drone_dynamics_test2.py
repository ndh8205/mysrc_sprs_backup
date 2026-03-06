#!/usr/bin/env python3
"""
Mars drone dynamics verification using gz.transport Python bindings.
Uses persistent publisher for reliable motor commands + simulation time.

Usage: (with simulation already running)
  python3 scripts/mars_drone_dynamics_test2.py
"""

import time
import subprocess
import re
import threading

from gz.transport13 import Node
from gz.msgs10 import actuators_pb2, world_stats_pb2


class SimMonitor:
    """Monitor simulation time via /world/.../stats topic."""

    def __init__(self, world_name="mars_drone_world"):
        self.node = Node()
        self.sim_time = 0.0
        self.rtf = 0.0
        topic = f"/world/{world_name}/stats"
        self.node.subscribe(world_stats_pb2.WorldStatistics, topic, self._cb)

    def _cb(self, msg):
        self.sim_time = msg.sim_time.sec + msg.sim_time.nsec * 1e-9
        self.rtf = msg.real_time_factor


class MotorController:
    """Persistent publisher for motor speed commands."""

    def __init__(self, model_name="x500_mars"):
        self.node = Node()
        topic = f"/{model_name}/command/motor_speed"
        self.pub = self.node.advertise(topic, actuators_pb2.Actuators)
        time.sleep(0.5)  # Wait for publisher discovery

    def set_speed(self, speeds):
        """Set motor speeds [w0, w1, w2, w3] in rad/s."""
        msg = actuators_pb2.Actuators()
        for s in speeds:
            msg.velocity.append(s)
        self.pub.publish(msg)

    def stop(self):
        self.set_speed([0, 0, 0, 0])


def get_pose():
    """Get x500_mars model pose from Gazebo."""
    result = subprocess.run(
        ['gz', 'model', '-m', 'x500_mars', '-p'],
        capture_output=True, text=True, timeout=5
    )
    lines = result.stdout.strip().split('\n')
    for i, line in enumerate(lines):
        if 'XYZ' in line and i + 1 < len(lines):
            match = re.findall(r'[-\d.]+', lines[i + 1])
            if len(match) >= 3:
                return float(match[0]), float(match[1]), float(match[2])
    return None


def reset_pose(x=11.99, y=-8.88, z=50.0):
    """Reset drone to upright pose at given position."""
    subprocess.run(
        ['gz', 'service', '-s', '/world/mars_drone_world/set_pose',
         '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '3000', '-r',
         f"name: 'x500_mars', position: {{x: {x}, y: {y}, z: {z}}}, "
         f"orientation: {{x: 0, y: 0, z: 0, w: 1}}"],
        capture_output=True, timeout=5
    )


def test_freefall(monitor):
    """Test 1: Free fall - should fall at Mars gravity (3.72 m/s^2)."""
    print("=" * 60)
    print("TEST 1: Free fall (no motor)")
    print("Expected: z(t) = z0 - 0.5 * 3.72 * t^2")
    print("=" * 60)

    reset_pose(z=50.0)
    time.sleep(1.0)

    t0 = monitor.sim_time
    readings = []
    for _ in range(8):
        p = get_pose()
        t = monitor.sim_time - t0
        if p:
            readings.append((t, p[2]))
        time.sleep(0.5)

    print(f"{'SimTime':>10} {'Z(m)':>10} {'dZ':>10} {'Expected_dZ':>12}")
    print("-" * 44)
    z0 = readings[0][1] if readings else 0
    t_base = readings[0][0] if readings else 0
    for t, z in readings:
        dt = t - t_base
        dz = z - z0
        expected_dz = -0.5 * 3.72 * dt ** 2
        print(f"{t:10.3f} {z:10.3f} {dz:10.3f} {expected_dz:12.3f}")

    if len(readings) >= 3:
        # Use last reading for overall check
        dt = readings[-1][0] - t_base
        dz = readings[-1][1] - z0
        if dt > 0:
            measured_g = -2 * dz / (dt ** 2)
            print(f"\nMeasured g = {measured_g:.3f} m/s^2 (expected: 3.72)")


def test_motor_thrust(motor_ctrl, monitor, motor_speed, label=""):
    """Test motor at a given speed with continuous publishing."""
    print("\n" + "=" * 60)
    thrust_per = 1.396e-07 * motor_speed ** 2
    total_thrust = 4 * thrust_per
    weight = 2.064 * 3.72
    net = total_thrust - weight
    expected_a = net / 2.064
    print(f"TEST: Motor speed = {motor_speed} rad/s {label}")
    print(f"  Thrust/motor = {thrust_per:.4f} N, Total = {total_thrust:.4f} N")
    print(f"  Weight       = {weight:.4f} N")
    print(f"  Net force    = {net:.4f} N → expected a = {expected_a:.4f} m/s^2")
    print("=" * 60)

    reset_pose(z=50.0)
    time.sleep(1.0)

    # Start continuous motor commands in a separate thread
    running = threading.Event()
    running.set()

    def motor_loop():
        while running.is_set():
            motor_ctrl.set_speed([motor_speed] * 4)
            time.sleep(0.01)  # 100 Hz command rate

    motor_thread = threading.Thread(target=motor_loop, daemon=True)
    motor_thread.start()

    t0 = monitor.sim_time
    readings = []
    for _ in range(12):
        p = get_pose()
        t = monitor.sim_time - t0
        if p:
            readings.append((t, p[2]))
        time.sleep(0.5)

    # Stop motors
    running.clear()
    motor_thread.join(timeout=1)
    motor_ctrl.stop()

    print(f"{'SimTime':>10} {'Z(m)':>10} {'dZ':>10}")
    print("-" * 32)
    z0 = readings[0][1] if readings else 0
    t_base = readings[0][0] if readings else 0
    for t, z in readings:
        dz = z - z0
        print(f"{t:10.3f} {z:10.3f} {dz:10.3f}")

    # Estimate effective acceleration
    if len(readings) >= 3:
        dt = readings[-1][0] - t_base
        dz = readings[-1][1] - z0
        if dt > 0:
            # z = z0 + v0*t + 0.5*a*t^2; assuming v0~0 after reset
            measured_a = 2 * dz / (dt ** 2)
            measured_thrust = (measured_a + 3.72) * 2.064
            measured_k = measured_thrust / (4 * motor_speed ** 2)
            print(f"\n  Measured accel    = {measured_a:.4f} m/s^2")
            print(f"  Measured thrust   = {measured_thrust:.4f} N")
            print(f"  Measured k (motor)= {measured_k:.4e} (set: 1.396e-07)")


def test_motor_off_after_on(motor_ctrl, monitor, motor_speed):
    """Test: motors on → off. Should switch from thrust to freefall."""
    print("\n" + "=" * 60)
    print(f"TEST: Motor {motor_speed} → 0 (cutoff)")
    print("=" * 60)

    reset_pose(z=50.0)
    time.sleep(1.0)

    # Phase 1: motors ON for 3 seconds
    running = threading.Event()
    running.set()

    def motor_loop():
        while running.is_set():
            motor_ctrl.set_speed([motor_speed] * 4)
            time.sleep(0.01)

    motor_thread = threading.Thread(target=motor_loop, daemon=True)
    motor_thread.start()

    print("Phase 1: Motors ON (3s)")
    t0 = monitor.sim_time
    readings_on = []
    for _ in range(6):
        p = get_pose()
        t = monitor.sim_time - t0
        if p:
            readings_on.append((t, p[2]))
        time.sleep(0.5)

    # Phase 2: motors OFF
    running.clear()
    motor_thread.join(timeout=1)
    motor_ctrl.stop()
    print("Phase 2: Motors OFF")

    readings_off = []
    for _ in range(6):
        p = get_pose()
        t = monitor.sim_time - t0
        if p:
            readings_off.append((t, p[2]))
        time.sleep(0.5)

    print(f"{'SimTime':>10} {'Z(m)':>10} {'Phase':>8}")
    print("-" * 30)
    for t, z in readings_on:
        print(f"{t:10.3f} {z:10.3f} {'ON':>8}")
    for t, z in readings_off:
        print(f"{t:10.3f} {z:10.3f} {'OFF':>8}")


if __name__ == '__main__':
    print("Mars Drone Dynamics Verification (v2 - gz.transport)")
    print(f"motorConstant: 1.396e-07 (Mars scaled)")
    print(f"Mass: ~2.064 kg, Mars g: 3.72 m/s^2")
    print(f"Hover speed: ~3709 rad/s")
    print()

    monitor = SimMonitor()
    motor = MotorController()

    # Wait for monitor to start receiving data
    time.sleep(1.0)
    print(f"Sim time: {monitor.sim_time:.2f}s, RTF: {monitor.rtf:.3f}")
    print()

    test_freefall(monitor)
    test_motor_thrust(motor, monitor, 3709, "(theoretical hover)")
    test_motor_thrust(motor, monitor, 4500, "(above hover)")
    test_motor_off_after_on(motor, monitor, 4500)

    motor.stop()
    print("\n" + "=" * 60)
    print("DONE")
    print("=" * 60)
