#!/usr/bin/env python3
"""
Mars drone dynamics verification script.
Sends motor commands and records pose to verify physics.

Usage:
  1. Launch simulation:  ros2 launch orbit_sim mars_drone.launch.py
  2. Run this script:    python3 scripts/mars_drone_dynamics_test.py
"""

import subprocess
import time
import re


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


def send_motor(speeds):
    """Send motor speed command (single publish)."""
    vel_str = f'velocity: [{speeds[0]}, {speeds[1]}, {speeds[2]}, {speeds[3]}]'
    subprocess.run(
        ['gz', 'topic', '-t', '/x500_mars/command/motor_speed',
         '-m', 'gz.msgs.Actuators', '-p', vel_str],
        capture_output=True, timeout=5
    )


def reset_pose(x=11.99, y=-8.88, z=6.0):
    """Reset drone to upright pose."""
    subprocess.run(
        ['gz', 'service', '-s', '/world/mars_drone_world/set_pose',
         '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '3000', '-r',
         f"name: 'x500_mars', position: {{x: {x}, y: {y}, z: {z}}}, "
         f"orientation: {{x: 0, y: 0, z: 0, w: 1}}"],
        capture_output=True, timeout=5
    )


def test_freefall():
    """Test 1: Free fall - should fall at Mars gravity (3.72 m/s^2)."""
    print("=" * 60)
    print("TEST 1: Free fall (no motor)")
    print("Expected: a = -3.72 m/s^2 (Mars gravity)")
    print("=" * 60)

    reset_pose(z=8.0)
    time.sleep(0.5)

    poses = []
    t0 = time.time()
    for _ in range(10):
        p = get_pose()
        if p:
            poses.append((time.time() - t0, p[2]))
        time.sleep(0.3)

    print(f"{'Time(s)':>8} {'Z(m)':>10} {'dZ':>10}")
    print("-" * 30)
    for i, (t, z) in enumerate(poses):
        dz = z - poses[0][1] if i > 0 else 0.0
        print(f"{t:8.2f} {z:10.3f} {dz:10.3f}")

    if len(poses) >= 2:
        dt = poses[-1][0] - poses[0][0]
        dz = poses[-1][1] - poses[0][1]
        avg_accel = 2 * dz / (dt ** 2) if dt > 0 else 0
        print(f"\nTotal dZ = {dz:.3f} m in {dt:.2f} s")
        print(f"Approx accel = {avg_accel:.2f} m/s^2 (expected: -3.72)")


def test_hover(motor_speed):
    """Test 2: Motor at given speed - check if hover/climb/descend."""
    print("\n" + "=" * 60)
    print(f"TEST 2: Motor speed = {motor_speed} rad/s")
    thrust_per_motor = 1.396e-07 * motor_speed ** 2
    total_thrust = 4 * thrust_per_motor
    weight = 2.064 * 3.72
    net = total_thrust - weight
    expected_a = net / 2.064
    print(f"Thrust/motor = {thrust_per_motor:.3f} N, Total = {total_thrust:.3f} N")
    print(f"Mars weight  = {weight:.3f} N")
    print(f"Net force    = {net:.3f} N → expected a = {expected_a:.3f} m/s^2")
    print("=" * 60)

    reset_pose(z=8.0)
    time.sleep(0.5)

    poses = []
    t0 = time.time()
    for _ in range(15):
        send_motor([motor_speed] * 4)
        time.sleep(0.05)
        send_motor([motor_speed] * 4)
        time.sleep(0.05)
        p = get_pose()
        if p:
            poses.append((time.time() - t0, p[0], p[1], p[2]))

    # Stop motors
    send_motor([0, 0, 0, 0])

    print(f"{'Time(s)':>8} {'Z(m)':>10} {'dZ':>10}")
    print("-" * 30)
    for i, (t, x, y, z) in enumerate(poses):
        dz = z - poses[0][3] if i > 0 else 0.0
        print(f"{t:8.2f} {z:10.3f} {dz:10.3f}")


def test_motor_off_after_on(motor_speed):
    """Test 3: Motors on then off - verify thrust stops."""
    print("\n" + "=" * 60)
    print(f"TEST 3: Motor {motor_speed} → 0 (thrust cutoff)")
    print("=" * 60)

    reset_pose(z=8.0)
    time.sleep(0.5)

    # Motors on for ~1.5s
    print("Phase 1: Motors ON")
    for _ in range(10):
        send_motor([motor_speed] * 4)
        time.sleep(0.1)

    p1 = get_pose()
    print(f"  Pose after thrust: Z = {p1[2]:.3f}" if p1 else "  Failed to get pose")

    # Motors off
    send_motor([0, 0, 0, 0])
    print("Phase 2: Motors OFF")
    time.sleep(1.5)

    p2 = get_pose()
    print(f"  Pose after 1.5s fall: Z = {p2[2]:.3f}" if p2 else "  Failed to get pose")

    if p1 and p2:
        dz = p2[2] - p1[2]
        print(f"  dZ = {dz:.3f} m (should be negative = falling)")


if __name__ == '__main__':
    print("Mars Drone Dynamics Verification")
    print(f"Motor constant: 1.396e-07 (Mars scaled)")
    print(f"Mass: ~2.064 kg, Mars g: 3.72 m/s^2")
    print(f"Hover speed: ~3709 rad/s")
    print()

    test_freefall()
    test_hover(3709)   # hover speed
    test_hover(4000)   # above hover
    test_motor_off_after_on(4000)

    print("\n" + "=" * 60)
    print("DONE")
    print("=" * 60)
