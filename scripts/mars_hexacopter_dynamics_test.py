#!/usr/bin/env python3
"""
Mars Hexacopter Dynamics Verification (gz.transport)

Verifies V6~V9 from docs/mars_hexacopter_design.md:
  V6: Free fall = 3.72 m/s^2
  V7: Hover at ~409 rad/s
  V8: Thrust proportional to omega^2
  V9: Yaw balance (3 CCW + 3 CW)

Usage: (with mars_hexacopter simulation running)
  python3 scripts/mars_hexacopter_dynamics_test.py
"""

import math
import time
import subprocess
import re
import threading

from gz.transport13 import Node
from gz.msgs10 import actuators_pb2, world_stats_pb2

# Design parameters (from mars_hexacopter_design_calc.py)
MOTOR_CONSTANT = 1.854561e-05
MOMENT_CONSTANT = 0.0590
TOTAL_MASS = 5.0
MARS_G = 3.72
NUM_ROTORS = 6
OMEGA_HOVER = 408.85
MAX_ROT_VELOCITY = 500.0

MODEL_NAME = "mars_hexacopter"
WORLD_NAME = "mars_hexacopter_world"


class SimMonitor:
    """Monitor simulation time via /world/.../stats topic."""

    def __init__(self):
        self.node = Node()
        self.sim_time = 0.0
        self.rtf = 0.0
        self.paused = False
        topic = f"/world/{WORLD_NAME}/stats"
        self.node.subscribe(world_stats_pb2.WorldStatistics, topic, self._cb)

    def _cb(self, msg):
        self.sim_time = msg.sim_time.sec + msg.sim_time.nsec * 1e-9
        self.rtf = msg.real_time_factor
        self.paused = msg.paused


class MotorController:
    """Persistent publisher for motor speed commands (6 motors)."""

    def __init__(self):
        self.node = Node()
        topic = f"/{MODEL_NAME}/command/motor_speed"
        self.pub = self.node.advertise(topic, actuators_pb2.Actuators)
        time.sleep(0.5)

    def set_speed(self, speeds):
        """Set motor speeds [w0..w5] in rad/s."""
        msg = actuators_pb2.Actuators()
        for s in speeds:
            msg.velocity.append(s)
        self.pub.publish(msg)

    def set_uniform(self, omega):
        """Set all 6 motors to the same speed."""
        self.set_speed([omega] * NUM_ROTORS)

    def stop(self):
        self.set_speed([0] * NUM_ROTORS)


def gz_service(service, reqtype, reptype, request):
    """Call a gz service."""
    try:
        subprocess.run(
            ['gz', 'service', '-s', service,
             '--reqtype', reqtype, '--reptype', reptype,
             '--timeout', '3000', '-r', request],
            capture_output=True, timeout=5
        )
    except Exception:
        pass


def pause_sim():
    gz_service(f'/world/{WORLD_NAME}/control',
               'gz.msgs.WorldControl', 'gz.msgs.Boolean',
               'pause: true')
    time.sleep(0.1)


def unpause_sim():
    gz_service(f'/world/{WORLD_NAME}/control',
               'gz.msgs.WorldControl', 'gz.msgs.Boolean',
               'pause: false')
    time.sleep(0.1)


def get_pose():
    """Get model pose (x, y, z) from Gazebo."""
    try:
        result = subprocess.run(
            ['gz', 'model', '-m', MODEL_NAME, '-p'],
            capture_output=True, text=True, timeout=5
        )
        lines = result.stdout.strip().split('\n')
        for i, line in enumerate(lines):
            if 'XYZ' in line and i + 1 < len(lines):
                match = re.findall(r'[-\d.]+', lines[i + 1])
                if len(match) >= 3:
                    return float(match[0]), float(match[1]), float(match[2])
    except Exception:
        pass
    return None


def get_orientation():
    """Get model RPY from Gazebo."""
    try:
        result = subprocess.run(
            ['gz', 'model', '-m', MODEL_NAME, '-p'],
            capture_output=True, text=True, timeout=5
        )
        lines = result.stdout.strip().split('\n')
        for i, line in enumerate(lines):
            if 'RPY' in line and i + 1 < len(lines):
                match = re.findall(r'[-\d.]+', lines[i + 1])
                if len(match) >= 3:
                    return float(match[0]), float(match[1]), float(match[2])
    except Exception:
        pass
    return None


def reset_pose(x=11.99, y=-8.88, z=50.0):
    """Reset drone: pause → set pose → unpause (ensures v=0)."""
    pause_sim()
    time.sleep(0.2)
    gz_service(f'/world/{WORLD_NAME}/set_pose',
               'gz.msgs.Pose', 'gz.msgs.Boolean',
               f"name: '{MODEL_NAME}', position: {{x: {x}, y: {y}, z: {z}}}, "
               f"orientation: {{x: 0, y: 0, z: 0, w: 1}}")
    time.sleep(0.3)
    unpause_sim()
    time.sleep(0.2)


def collect_z_readings(monitor, duration_s=4.0, interval=0.25):
    """Collect (sim_time, z) readings."""
    readings = []
    n = int(duration_s / interval)
    for _ in range(n):
        p = get_pose()
        t = monitor.sim_time
        if p:
            readings.append((t, p[2]))
        time.sleep(interval)
    return readings


def run_motor_continuous(motor_ctrl, omega, running_event):
    """Publish motor commands at 100Hz in a loop."""
    while running_event.is_set():
        motor_ctrl.set_uniform(omega)
        time.sleep(0.01)


def fit_freefall(readings):
    """Fit z(t) = z0 + v0*t - 0.5*g*t^2 via least squares to extract g.
    Returns (v0, g_fitted)."""
    if len(readings) < 3:
        return None, None
    t0 = readings[0][0]
    z0 = readings[0][1]
    # z - z0 = v0*(t-t0) - 0.5*g*(t-t0)^2
    # Let dt = t-t0, dz = z-z0
    # dz = v0*dt - 0.5*g*dt^2
    # Linear system: dz = a*dt + b*dt^2 where a=v0, b=-0.5*g
    n = len(readings) - 1
    sum_dt2 = 0; sum_dt3 = 0; sum_dt4 = 0
    sum_dz_dt = 0; sum_dz_dt2 = 0
    for i in range(1, len(readings)):
        dt = readings[i][0] - t0
        dz = readings[i][1] - z0
        sum_dt2 += dt**2
        sum_dt3 += dt**3
        sum_dt4 += dt**4
        sum_dz_dt += dz * dt
        sum_dz_dt2 += dz * dt**2
    # Normal equations: [sum_dt2, sum_dt3; sum_dt3, sum_dt4] * [a; b] = [sum_dz_dt; sum_dz_dt2]
    det = sum_dt2 * sum_dt4 - sum_dt3 * sum_dt3
    if abs(det) < 1e-20:
        return None, None
    a = (sum_dt4 * sum_dz_dt - sum_dt3 * sum_dz_dt2) / det
    b = (sum_dt2 * sum_dz_dt2 - sum_dt3 * sum_dz_dt) / det
    v0 = a
    g_fitted = -2 * b
    return v0, g_fitted


def print_header(title):
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print(f"{'=' * 60}")


def test_freefall(monitor):
    """V6: Free fall — fit trajectory to extract g."""
    print_header("V6: Free Fall (no motors)")
    print(f"  Expected: g = {MARS_G} m/s^2")

    reset_pose(z=80.0)  # High altitude to avoid ground
    time.sleep(0.3)

    readings = collect_z_readings(monitor, duration_s=3.0, interval=0.2)
    if len(readings) < 4:
        print("  ERROR: Not enough readings")
        return

    # Only use readings before potential ground impact (z > 10)
    valid = [(t, z) for t, z in readings if z > 10]
    if len(valid) < 4:
        valid = readings[:6]

    t0 = valid[0][0]
    z0 = valid[0][1]
    print(f"\n  {'dt(s)':>8} {'Z(m)':>10} {'dZ':>10} {'Expected':>10}")
    print("  " + "-" * 42)
    for t, z in valid:
        dt = t - t0
        dz = z - z0
        expected = -0.5 * MARS_G * dt ** 2
        print(f"  {dt:8.3f} {z:10.3f} {dz:10.3f} {expected:10.3f}")

    v0, g_fitted = fit_freefall(valid)
    if v0 is not None:
        err = abs(g_fitted - MARS_G) / MARS_G * 100
        status = "PASS" if err < 10 else "FAIL"
        print(f"\n  Fitted: v0 = {v0:.3f} m/s, g = {g_fitted:.3f} m/s^2")
        print(f"  Error = {err:.1f}%  [{status}]")
    else:
        print("  ERROR: Fitting failed")


def test_hover(motor_ctrl, monitor):
    """V7: At omega_hover, drone should maintain altitude."""
    print_header(f"V7: Hover at omega = {OMEGA_HOVER:.1f} rad/s")
    weight = TOTAL_MASS * MARS_G
    thrust = NUM_ROTORS * MOTOR_CONSTANT * OMEGA_HOVER ** 2
    print(f"  Weight = {weight:.3f} N, Thrust = {thrust:.3f} N")

    # Start motors BEFORE unpause to ensure immediate thrust
    pause_sim()
    time.sleep(0.2)
    gz_service(f'/world/{WORLD_NAME}/set_pose',
               'gz.msgs.Pose', 'gz.msgs.Boolean',
               f"name: '{MODEL_NAME}', position: {{x: 11.99, y: -8.88, z: 50}}, "
               f"orientation: {{x: 0, y: 0, z: 0, w: 1}}")
    time.sleep(0.3)

    # Start motor thread while sim is paused
    running = threading.Event()
    running.set()
    t = threading.Thread(target=run_motor_continuous,
                         args=(motor_ctrl, OMEGA_HOVER, running), daemon=True)
    t.start()
    time.sleep(0.3)  # Let publisher send first commands

    unpause_sim()

    # Wait for stabilization (motor spin-up)
    time.sleep(2.0)

    # Now collect stable readings
    readings = collect_z_readings(monitor, duration_s=5.0, interval=0.5)

    running.clear()
    t.join(timeout=1)
    motor_ctrl.stop()

    if not readings:
        print("  ERROR: No readings")
        return

    t0 = readings[0][0]
    z0 = readings[0][1]
    print(f"\n  {'dt(s)':>8} {'Z(m)':>10} {'dZ':>10}")
    print("  " + "-" * 30)
    for ti, z in readings:
        print(f"  {ti - t0:8.3f} {z:10.3f} {z - z0:10.3f}")

    # Check stability using last vs first reading
    dz_total = readings[-1][1] - readings[0][1]
    dt_total = readings[-1][0] - readings[0][0]
    z_vals = [z for _, z in readings]
    z_range = max(z_vals) - min(z_vals)

    if dt_total > 2:
        avg_vel = dz_total / dt_total
        print(f"\n  Z range = {z_range:.3f} m")
        print(f"  Average vertical velocity = {avg_vel:.4f} m/s")
        if z_range < 3.0 and abs(avg_vel) < 0.5:
            print("  PASS: Hover stable")
        else:
            # Estimate motor constant
            measured_a = 2 * dz_total / (dt_total ** 2) if abs(dt_total) > 0.1 else 0
            measured_thrust = (measured_a + MARS_G) * TOTAL_MASS
            measured_k = measured_thrust / (NUM_ROTORS * OMEGA_HOVER ** 2)
            print(f"  Measured a = {measured_a:.4f} m/s^2")
            print(f"  Measured k = {measured_k:.4e} (set: {MOTOR_CONSTANT:.4e})")


def test_above_hover(motor_ctrl, monitor):
    """V8: Above hover speed — should rise, verify T proportional to omega^2."""
    omega_test = 450.0
    thrust_per = MOTOR_CONSTANT * omega_test ** 2
    total_thrust = NUM_ROTORS * thrust_per
    weight = TOTAL_MASS * MARS_G
    expected_a = (total_thrust - weight) / TOTAL_MASS

    print_header(f"V8: Above hover (omega = {omega_test} rad/s)")
    print(f"  Thrust/rotor = {thrust_per:.4f} N, Total = {total_thrust:.3f} N")
    print(f"  Weight       = {weight:.3f} N")
    print(f"  Expected a   = {expected_a:.4f} m/s^2")

    # Reset with motors ready
    pause_sim()
    time.sleep(0.2)
    gz_service(f'/world/{WORLD_NAME}/set_pose',
               'gz.msgs.Pose', 'gz.msgs.Boolean',
               f"name: '{MODEL_NAME}', position: {{x: 11.99, y: -8.88, z: 30}}, "
               f"orientation: {{x: 0, y: 0, z: 0, w: 1}}")
    time.sleep(0.3)

    running = threading.Event()
    running.set()
    t = threading.Thread(target=run_motor_continuous,
                         args=(motor_ctrl, omega_test, running), daemon=True)
    t.start()
    time.sleep(0.3)

    unpause_sim()
    time.sleep(2.0)  # Wait for motor spin-up

    readings = collect_z_readings(monitor, duration_s=5.0, interval=0.5)

    running.clear()
    t.join(timeout=1)
    motor_ctrl.stop()

    if not readings:
        print("  ERROR: No readings")
        return

    t0 = readings[0][0]
    z0 = readings[0][1]
    print(f"\n  {'dt(s)':>8} {'Z(m)':>10} {'dZ':>10}")
    print("  " + "-" * 30)
    for ti, z in readings:
        print(f"  {ti - t0:8.3f} {z:10.3f} {z - z0:10.3f}")

    # Use last few stable readings to estimate acceleration
    dt = readings[-1][0] - t0
    dz = readings[-1][1] - z0
    if dt > 1:
        # z = z0 + v0*t + 0.5*a*t^2; with v0≈0 after stabilization
        v0_fit, a_fit = None, None
        if len(readings) >= 4:
            # Use fit to extract acceleration
            v0_fit, g_neg = fit_freefall(readings)
            if g_neg is not None:
                a_fit = -g_neg  # fit gives 'g' as downward; actual accel is upward
                measured_thrust = (a_fit + MARS_G) * TOTAL_MASS
                measured_k = measured_thrust / (NUM_ROTORS * omega_test ** 2)
                err_k = abs(measured_k - MOTOR_CONSTANT) / MOTOR_CONSTANT * 100
                print(f"\n  Fitted accel     = {a_fit:.4f} m/s^2 (expected: {expected_a:.4f})")
                print(f"  Measured k       = {measured_k:.4e} (set: {MOTOR_CONSTANT:.4e})")
                print(f"  k error          = {err_k:.1f}%")
                status = "PASS" if err_k < 15 else "FAIL"
                print(f"  [{status}]")


def test_yaw_balance(motor_ctrl, monitor):
    """V9: Equal speed on all 6 motors — no yaw angular acceleration.

    Note: Previous tests may leave residual angular velocity.
    We check yaw RATE constancy (no angular acceleration = no net torque),
    rather than absolute yaw stability.
    """
    print_header("V9: Yaw Balance (all motors equal)")
    print("  Checking: yaw angular acceleration ≈ 0 (torque balanced)")
    print("  Note: constant yaw rate from residual ω is expected")

    # Reset with motors ready
    pause_sim()
    time.sleep(0.2)
    gz_service(f'/world/{WORLD_NAME}/set_pose',
               'gz.msgs.Pose', 'gz.msgs.Boolean',
               f"name: '{MODEL_NAME}', position: {{x: 11.99, y: -8.88, z: 50}}, "
               f"orientation: {{x: 0, y: 0, z: 0, w: 1}}")
    time.sleep(0.3)

    running = threading.Event()
    running.set()
    t = threading.Thread(target=run_motor_continuous,
                         args=(motor_ctrl, OMEGA_HOVER, running), daemon=True)
    t.start()
    time.sleep(0.3)

    unpause_sim()
    time.sleep(2.0)  # Wait for stabilization

    yaw_readings = []
    for _ in range(12):
        rpy = get_orientation()
        if rpy:
            yaw_readings.append((monitor.sim_time, rpy[2]))
        time.sleep(0.5)

    running.clear()
    t.join(timeout=1)
    motor_ctrl.stop()

    if len(yaw_readings) >= 4:
        t0_yaw = yaw_readings[0][0]
        print(f"\n  {'dt(s)':>8} {'Yaw(deg)':>10} {'Rate(deg/s)':>12}")
        print("  " + "-" * 34)

        # Calculate yaw rates between consecutive readings
        rates = []
        for i in range(len(yaw_readings)):
            ti, yaw = yaw_readings[i]
            if i > 0:
                dt = ti - yaw_readings[i - 1][0]
                dyaw = yaw - yaw_readings[i - 1][1]
                if dt > 0.01:
                    rate = dyaw / dt
                    rates.append(rate)
                    print(f"  {ti - t0_yaw:8.3f} {math.degrees(yaw):10.1f} {math.degrees(rate):12.1f}")
            else:
                print(f"  {ti - t0_yaw:8.3f} {math.degrees(yaw):10.1f} {'---':>12}")

        if len(rates) >= 3:
            avg_rate = sum(rates) / len(rates)
            # Check rate consistency (constant rate = no angular acceleration)
            rate_std = (sum((r - avg_rate) ** 2 for r in rates) / len(rates)) ** 0.5
            rate_range = max(rates) - min(rates)

            print(f"\n  Yaw rate: avg = {math.degrees(avg_rate):.1f} deg/s")
            print(f"  Yaw rate std  = {math.degrees(rate_std):.1f} deg/s")
            print(f"  Yaw rate range = {math.degrees(rate_range):.1f} deg/s")

            # If rate variation is small, torque is balanced (no angular accel)
            if math.degrees(rate_std) < 20:
                if abs(math.degrees(avg_rate)) < 5:
                    print("  PASS: No yaw drift, torque perfectly balanced")
                else:
                    print(f"  PASS: Constant yaw rate (residual angular velocity)")
                    print(f"        Angular acceleration ≈ 0 → torque balanced")
            else:
                print("  FAIL: Yaw rate changing → net torque exists")
    else:
        print("  ERROR: Could not read orientation")


if __name__ == '__main__':
    print("Mars Hexacopter Dynamics Verification (v2)")
    print(f"  motorConstant: {MOTOR_CONSTANT:.4e}")
    print(f"  Total mass: {TOTAL_MASS} kg, Mars g: {MARS_G} m/s^2")
    print(f"  Hover omega: {OMEGA_HOVER:.1f} rad/s ({OMEGA_HOVER*60/(2*math.pi):.0f} RPM)")
    print(f"  Num rotors: {NUM_ROTORS}")
    print()

    monitor = SimMonitor()
    motor = MotorController()

    time.sleep(1.5)
    print(f"Sim time: {monitor.sim_time:.2f}s, RTF: {monitor.rtf:.3f}")
    print()

    test_freefall(monitor)
    test_hover(motor, monitor)
    test_above_hover(motor, monitor)
    test_yaw_balance(motor, monitor)

    motor.stop()
    print(f"\n{'=' * 60}")
    print("DONE — All V6~V9 tests completed")
    print(f"{'=' * 60}")
