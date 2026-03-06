#!/usr/bin/env python3
"""
Mars Hexacopter + CMG Dynamics Verification (gz.transport)

Tests:
  T1: Hover stability with CMG (motors only, CMG idle)
  T2: CMG wheel spin-up (no gyroscopic effect while gimbal locked at 0)
  T3: Gyroscopic precession (wheel spinning + gimbal tilt → body pitch response)
  T4: CMG yaw authority (differential gimbal → yaw torque)

Usage: (with mars_hexacopter simulation running)
  python3 scripts/mars_hexacopter_cmg_test.py
"""

import math
import time
import subprocess
import re
import threading

from gz.transport13 import Node
from gz.msgs10 import actuators_pb2, world_stats_pb2, double_pb2

# Design parameters
MOTOR_CONSTANT = 1.854561e-05
TOTAL_MASS = 5.0
MARS_G = 3.72
NUM_ROTORS = 6
OMEGA_HOVER = 408.85

# CMG parameters (from cubesat_cmg / MATLAB CMG_sim.m)
CMG_WHEEL_INERTIA = 3.65e-5  # Ixx of wheel (spin axis)
CMG_NOMINAL_RPM = 10000
CMG_NOMINAL_RAD = CMG_NOMINAL_RPM * 2 * math.pi / 60  # ~1047 rad/s

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

    def set_uniform(self, omega):
        msg = actuators_pb2.Actuators()
        for _ in range(NUM_ROTORS):
            msg.velocity.append(omega)
        self.pub.publish(msg)

    def stop(self):
        msg = actuators_pb2.Actuators()
        for _ in range(NUM_ROTORS):
            msg.velocity.append(0)
        self.pub.publish(msg)


class CMGController:
    """Publisher for CMG gimbal/wheel velocity commands via gz.transport."""

    def __init__(self):
        self.node = Node()
        self.gimbal1_pub = self.node.advertise(
            f"/{MODEL_NAME}/cmg1_gimbal/cmd_vel", double_pb2.Double)
        self.gimbal2_pub = self.node.advertise(
            f"/{MODEL_NAME}/cmg2_gimbal/cmd_vel", double_pb2.Double)
        self.wheel1_pub = self.node.advertise(
            f"/{MODEL_NAME}/cmg1_wheel/cmd_vel", double_pb2.Double)
        self.wheel2_pub = self.node.advertise(
            f"/{MODEL_NAME}/cmg2_wheel/cmd_vel", double_pb2.Double)
        time.sleep(0.5)

    def set_wheels(self, w1_rad, w2_rad):
        msg1 = double_pb2.Double()
        msg1.data = w1_rad
        self.wheel1_pub.publish(msg1)
        msg2 = double_pb2.Double()
        msg2.data = w2_rad
        self.wheel2_pub.publish(msg2)

    def set_gimbals(self, g1_vel, g2_vel):
        msg1 = double_pb2.Double()
        msg1.data = g1_vel
        self.gimbal1_pub.publish(msg1)
        msg2 = double_pb2.Double()
        msg2.data = g2_vel
        self.gimbal2_pub.publish(msg2)

    def stop_all(self):
        self.set_wheels(0, 0)
        self.set_gimbals(0, 0)


def gz_service(service, reqtype, reptype, request):
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
    pause_sim()
    time.sleep(0.2)
    gz_service(f'/world/{WORLD_NAME}/set_pose',
               'gz.msgs.Pose', 'gz.msgs.Boolean',
               f"name: '{MODEL_NAME}', position: {{x: {x}, y: {y}, z: {z}}}, "
               f"orientation: {{x: 0, y: 0, z: 0, w: 1}}")
    time.sleep(0.3)
    unpause_sim()
    time.sleep(0.2)


def run_motor_continuous(motor_ctrl, omega, running_event):
    while running_event.is_set():
        motor_ctrl.set_uniform(omega)
        time.sleep(0.01)


def run_cmg_wheels_continuous(cmg_ctrl, w1, w2, running_event):
    while running_event.is_set():
        cmg_ctrl.set_wheels(w1, w2)
        time.sleep(0.01)


def run_cmg_gimbals_continuous(cmg_ctrl, g1, g2, running_event):
    while running_event.is_set():
        cmg_ctrl.set_gimbals(g1, g2)
        time.sleep(0.01)


def print_header(title):
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print(f"{'=' * 60}")


def test_hover_with_cmg(motor_ctrl, cmg_ctrl, monitor):
    """T1: Hover with CMG present but idle — should still hover at omega_hover."""
    print_header("T1: Hover with CMG (motors only, CMG idle)")
    print(f"  Motor omega = {OMEGA_HOVER:.1f} rad/s, CMG wheels/gimbals = 0")

    pause_sim()
    time.sleep(0.2)
    gz_service(f'/world/{WORLD_NAME}/set_pose',
               'gz.msgs.Pose', 'gz.msgs.Boolean',
               f"name: '{MODEL_NAME}', position: {{x: 11.99, y: -8.88, z: 50}}, "
               f"orientation: {{x: 0, y: 0, z: 0, w: 1}}")
    time.sleep(0.3)

    # CMG idle
    cmg_ctrl.stop_all()

    # Start motors
    running = threading.Event()
    running.set()
    t = threading.Thread(target=run_motor_continuous,
                         args=(motor_ctrl, OMEGA_HOVER, running), daemon=True)
    t.start()
    time.sleep(0.3)

    unpause_sim()
    time.sleep(2.0)

    # Collect readings
    readings = []
    for _ in range(10):
        p = get_pose()
        if p:
            readings.append((monitor.sim_time, p[2]))
        time.sleep(0.5)

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

    dz_total = readings[-1][1] - readings[0][1]
    dt_total = readings[-1][0] - readings[0][0]
    z_vals = [z for _, z in readings]
    z_range = max(z_vals) - min(z_vals)

    if dt_total > 2:
        avg_vel = dz_total / dt_total
        print(f"\n  Z range = {z_range:.3f} m")
        print(f"  Average vertical velocity = {avg_vel:.4f} m/s")
        if z_range < 3.0 and abs(avg_vel) < 0.5:
            print("  PASS: Hover stable with CMG on board")
        else:
            print("  FAIL: Hover not stable")


def test_wheel_spinup(motor_ctrl, cmg_ctrl, monitor):
    """T2: Spin up CMG wheels while hovering — no gyro effect (gimbal at 0)."""
    print_header("T2: CMG Wheel Spin-up (hover + wheel spin, gimbal=0)")
    print(f"  Wheel target: +/-{CMG_NOMINAL_RPM} RPM ({CMG_NOMINAL_RAD:.0f} rad/s)")

    pause_sim()
    time.sleep(0.2)
    gz_service(f'/world/{WORLD_NAME}/set_pose',
               'gz.msgs.Pose', 'gz.msgs.Boolean',
               f"name: '{MODEL_NAME}', position: {{x: 11.99, y: -8.88, z: 50}}, "
               f"orientation: {{x: 0, y: 0, z: 0, w: 1}}")
    time.sleep(0.3)

    # Start motors + CMG wheels (counter-rotating: +w, -w)
    motor_running = threading.Event()
    motor_running.set()
    mt = threading.Thread(target=run_motor_continuous,
                          args=(motor_ctrl, OMEGA_HOVER, motor_running), daemon=True)
    mt.start()

    wheel_running = threading.Event()
    wheel_running.set()
    wt = threading.Thread(target=run_cmg_wheels_continuous,
                          args=(cmg_ctrl, CMG_NOMINAL_RAD, -CMG_NOMINAL_RAD, wheel_running),
                          daemon=True)
    wt.start()

    time.sleep(0.3)
    unpause_sim()

    # Wait for spin-up + stabilize
    time.sleep(3.0)

    # Check attitude — should remain level
    rpy_readings = []
    z_readings = []
    for _ in range(8):
        rpy = get_orientation()
        p = get_pose()
        if rpy and p:
            rpy_readings.append((monitor.sim_time, rpy))
            z_readings.append((monitor.sim_time, p[2]))
        time.sleep(0.5)

    motor_running.clear()
    wheel_running.clear()
    mt.join(timeout=1)
    wt.join(timeout=1)
    motor_ctrl.stop()
    cmg_ctrl.stop_all()

    if rpy_readings:
        t0 = rpy_readings[0][0]
        print(f"\n  {'dt(s)':>8} {'Roll':>8} {'Pitch':>8} {'Yaw':>8} {'Z(m)':>8}")
        print("  " + "-" * 44)
        for i, (ti, rpy) in enumerate(rpy_readings):
            z = z_readings[i][1] if i < len(z_readings) else 0
            print(f"  {ti - t0:8.3f} {math.degrees(rpy[0]):8.2f} "
                  f"{math.degrees(rpy[1]):8.2f} {math.degrees(rpy[2]):8.2f} {z:8.2f}")

        # Check: roll/pitch should remain small
        max_roll = max(abs(math.degrees(rpy[0])) for _, rpy in rpy_readings)
        max_pitch = max(abs(math.degrees(rpy[1])) for _, rpy in rpy_readings)
        z_range = max(z for _, z in z_readings) - min(z for _, z in z_readings)

        print(f"\n  Max |roll|  = {max_roll:.2f} deg")
        print(f"  Max |pitch| = {max_pitch:.2f} deg")
        print(f"  Z range     = {z_range:.3f} m")

        if max_roll < 5 and max_pitch < 5 and z_range < 3:
            print("  PASS: Wheel spin-up does not disturb hover")
        else:
            print("  FAIL: Wheel spin-up disturbed attitude")
    else:
        print("  ERROR: No orientation readings")


def test_gyroscopic_precession(motor_ctrl, cmg_ctrl, monitor):
    """T3: Wheel spinning + gimbal tilt → gyroscopic torque on body.

    Both wheels spin same direction (+w, +w).
    Both gimbals tilt at same rate → net pitch torque on body.
    Expected: τ = 2 × h × gimbal_rate, where h = I_wheel × ω_wheel
    """
    print_header("T3: Gyroscopic Precession (gimbal tilt → body pitch)")

    h_wheel = CMG_WHEEL_INERTIA * CMG_NOMINAL_RAD
    gimbal_rate = 0.5  # rad/s
    expected_torque = 2 * h_wheel * gimbal_rate
    print(f"  Wheel h = {h_wheel:.4e} Nms")
    print(f"  Gimbal rate = {gimbal_rate:.1f} rad/s")
    print(f"  Expected torque = {expected_torque:.4e} Nm")
    print(f"  Body Iyy ≈ 0.396 → expected pitch accel ≈ {expected_torque/0.396:.4f} rad/s²")

    pause_sim()
    time.sleep(0.2)
    gz_service(f'/world/{WORLD_NAME}/set_pose',
               'gz.msgs.Pose', 'gz.msgs.Boolean',
               f"name: '{MODEL_NAME}', position: {{x: 11.99, y: -8.88, z: 50}}, "
               f"orientation: {{x: 0, y: 0, z: 0, w: 1}}")
    time.sleep(0.3)

    # Start motors + wheels (SAME direction for net effect)
    motor_running = threading.Event()
    motor_running.set()
    mt = threading.Thread(target=run_motor_continuous,
                          args=(motor_ctrl, OMEGA_HOVER, motor_running), daemon=True)
    mt.start()

    wheel_running = threading.Event()
    wheel_running.set()
    wt = threading.Thread(target=run_cmg_wheels_continuous,
                          args=(cmg_ctrl, CMG_NOMINAL_RAD, CMG_NOMINAL_RAD, wheel_running),
                          daemon=True)
    wt.start()

    time.sleep(0.3)
    unpause_sim()

    # Wait for wheels to spin up
    time.sleep(5.0)

    # Record baseline pitch
    rpy0 = get_orientation()
    t0 = monitor.sim_time
    if not rpy0:
        print("  ERROR: Cannot read orientation")
        motor_running.clear()
        wheel_running.clear()
        return

    pitch0 = rpy0[1]
    print(f"\n  Baseline pitch = {math.degrees(pitch0):.2f} deg at t = {t0:.2f}s")
    print("  Tilting gimbals...")

    # Tilt both gimbals at same rate for 2 seconds
    gimbal_running = threading.Event()
    gimbal_running.set()
    gt = threading.Thread(target=run_cmg_gimbals_continuous,
                          args=(cmg_ctrl, gimbal_rate, gimbal_rate, gimbal_running),
                          daemon=True)
    gt.start()

    # Collect pitch during tilt
    pitch_readings = []
    for _ in range(10):
        time.sleep(0.3)
        rpy = get_orientation()
        if rpy:
            pitch_readings.append((monitor.sim_time, rpy[1]))

    gimbal_running.clear()
    gt.join(timeout=1)
    cmg_ctrl.set_gimbals(0, 0)

    # Collect a few more readings after stopping gimbal
    for _ in range(4):
        time.sleep(0.3)
        rpy = get_orientation()
        if rpy:
            pitch_readings.append((monitor.sim_time, rpy[1]))

    motor_running.clear()
    wheel_running.clear()
    mt.join(timeout=1)
    wt.join(timeout=1)
    motor_ctrl.stop()
    cmg_ctrl.stop_all()

    if pitch_readings:
        print(f"\n  {'dt(s)':>8} {'Pitch(deg)':>12} {'dPitch':>10}")
        print("  " + "-" * 34)
        for ti, pitch in pitch_readings:
            dp = math.degrees(pitch - pitch0)
            print(f"  {ti - t0:8.3f} {math.degrees(pitch):12.3f} {dp:10.3f}")

        # Check if pitch changed meaningfully
        max_dp = max(abs(math.degrees(p - pitch0)) for _, p in pitch_readings)
        if max_dp > 0.5:
            print(f"\n  Max pitch change = {max_dp:.2f} deg")
            print("  PASS: Gyroscopic precession detected")
        else:
            print(f"\n  Max pitch change = {max_dp:.2f} deg (too small)")
            print("  FAIL: No gyroscopic response")
    else:
        print("  ERROR: No pitch readings")


def test_cmg_yaw_control(motor_ctrl, cmg_ctrl, monitor):
    """T4: CMG yaw authority — differential gimbal to generate yaw torque.

    Wheels counter-rotating: CMG1=+w, CMG2=-w (balanced angular momentum).
    Gimbal both in same direction → net yaw torque.
    """
    print_header("T4: CMG Yaw Control (differential wheel + same gimbal)")

    h_wheel = CMG_WHEEL_INERTIA * CMG_NOMINAL_RAD
    gimbal_rate = 0.5
    # With counter-rotating wheels and same gimbal tilt:
    # τ_yaw = 2 × h × gimbal_rate (both contribute same sign yaw torque)
    expected_torque = 2 * h_wheel * gimbal_rate
    print(f"  Wheel h = {h_wheel:.4e} Nms (counter-rotating)")
    print(f"  Gimbal rate = {gimbal_rate:.1f} rad/s (both same direction)")
    print(f"  Expected yaw torque = {expected_torque:.4e} Nm")
    print(f"  Body Izz ≈ 0.787 → expected yaw accel ≈ {expected_torque/0.787:.4f} rad/s²")

    pause_sim()
    time.sleep(0.2)
    gz_service(f'/world/{WORLD_NAME}/set_pose',
               'gz.msgs.Pose', 'gz.msgs.Boolean',
               f"name: '{MODEL_NAME}', position: {{x: 11.99, y: -8.88, z: 50}}, "
               f"orientation: {{x: 0, y: 0, z: 0, w: 1}}")
    time.sleep(0.3)

    # Start motors + counter-rotating wheels
    motor_running = threading.Event()
    motor_running.set()
    mt = threading.Thread(target=run_motor_continuous,
                          args=(motor_ctrl, OMEGA_HOVER, motor_running), daemon=True)
    mt.start()

    wheel_running = threading.Event()
    wheel_running.set()
    wt = threading.Thread(target=run_cmg_wheels_continuous,
                          args=(cmg_ctrl, CMG_NOMINAL_RAD, -CMG_NOMINAL_RAD, wheel_running),
                          daemon=True)
    wt.start()

    time.sleep(0.3)
    unpause_sim()
    time.sleep(5.0)  # Wait for wheel spin-up

    rpy0 = get_orientation()
    t0 = monitor.sim_time
    if not rpy0:
        print("  ERROR: Cannot read orientation")
        motor_running.clear()
        wheel_running.clear()
        return

    yaw0 = rpy0[2]
    print(f"\n  Baseline yaw = {math.degrees(yaw0):.2f} deg at t = {t0:.2f}s")
    print("  Tilting gimbals (same direction)...")

    # Tilt gimbals same direction
    gimbal_running = threading.Event()
    gimbal_running.set()
    gt = threading.Thread(target=run_cmg_gimbals_continuous,
                          args=(cmg_ctrl, gimbal_rate, gimbal_rate, gimbal_running),
                          daemon=True)
    gt.start()

    yaw_readings = []
    for _ in range(10):
        time.sleep(0.3)
        rpy = get_orientation()
        if rpy:
            yaw_readings.append((monitor.sim_time, rpy[2]))

    gimbal_running.clear()
    gt.join(timeout=1)
    cmg_ctrl.set_gimbals(0, 0)

    for _ in range(4):
        time.sleep(0.3)
        rpy = get_orientation()
        if rpy:
            yaw_readings.append((monitor.sim_time, rpy[2]))

    motor_running.clear()
    wheel_running.clear()
    mt.join(timeout=1)
    wt.join(timeout=1)
    motor_ctrl.stop()
    cmg_ctrl.stop_all()

    if yaw_readings:
        print(f"\n  {'dt(s)':>8} {'Yaw(deg)':>10} {'dYaw':>10}")
        print("  " + "-" * 32)
        for ti, yaw in yaw_readings:
            dy = math.degrees(yaw - yaw0)
            print(f"  {ti - t0:8.3f} {math.degrees(yaw):10.3f} {dy:10.3f}")

        max_dy = max(abs(math.degrees(y - yaw0)) for _, y in yaw_readings)
        if max_dy > 0.5:
            print(f"\n  Max yaw change = {max_dy:.2f} deg")
            print("  PASS: CMG yaw authority confirmed")
        else:
            print(f"\n  Max yaw change = {max_dy:.2f} deg (too small)")
            print("  FAIL: No CMG yaw response")
    else:
        print("  ERROR: No yaw readings")


if __name__ == '__main__':
    print("Mars Hexacopter + CMG Dynamics Verification")
    print(f"  Motor: k={MOTOR_CONSTANT:.4e}, hover_omega={OMEGA_HOVER:.1f} rad/s")
    print(f"  CMG: I_wheel={CMG_WHEEL_INERTIA:.2e}, nominal={CMG_NOMINAL_RPM} RPM")
    print(f"  CMG h = {CMG_WHEEL_INERTIA * CMG_NOMINAL_RAD:.4e} Nms per wheel")
    print()

    monitor = SimMonitor()
    motor = MotorController()
    cmg = CMGController()

    time.sleep(1.5)
    print(f"Sim time: {monitor.sim_time:.2f}s, RTF: {monitor.rtf:.3f}")
    print()

    test_hover_with_cmg(motor, cmg, monitor)
    test_wheel_spinup(motor, cmg, monitor)
    test_gyroscopic_precession(motor, cmg, monitor)
    test_cmg_yaw_control(motor, cmg, monitor)

    motor.stop()
    cmg.stop_all()
    print(f"\n{'=' * 60}")
    print("DONE — All T1~T4 CMG tests completed")
    print(f"{'=' * 60}")
