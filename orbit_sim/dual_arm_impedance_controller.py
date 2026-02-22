#!/usr/bin/env python3
"""
dual_arm_impedance_controller.py
Task-space Impedance Controller for KARI Dual ARM (Free-floating)

Left/Right 독립 제어 방식

Subscribes:
    /joint_states (sensor_msgs/JointState)
    /target_pose_L (geometry_msgs/PoseStamped) - Left EE 목표
    /target_pose_R (geometry_msgs/PoseStamped) - Right EE 목표
    /world/docking_world/pose/info (tf2_msgs/TFMessage) - 위성 pose

Publishes:
    /effort_controller_L/commands (std_msgs/Float64MultiArray)
    /effort_controller_R/commands (std_msgs/Float64MultiArray)

Author: Space Challenge Project
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage
import numpy as np

from orbit_sim.kari_dual_arm_dynamics import (
    KariArmParams, forward_kinematics, compute_mass_matrix,
    compute_generalized_jacobian, compute_coriolis,
    motor_dynamics, quat_to_dcm, dcm_to_quat, quat_mult, quat_inv
)


class ArmState:
    """단일 팔 상태"""
    def __init__(self, arm_id: str):
        self.arm_id = arm_id
        self.params = KariArmParams(arm_id=arm_id)
        
        self.theta = np.zeros(7)
        self.theta_dot = np.zeros(7)
        self.tau_actual = np.zeros(7)
        
        self.target_position = None
        self.target_orientation = None
        
        self.initialized = False


class DualArmImpedanceController(Node):
    def __init__(self):
        super().__init__('dual_arm_impedance_controller')
        
        # 임피던스 게인
        self.declare_parameter('K_pos', 50.0)
        self.declare_parameter('D_pos', 20.0)
        self.declare_parameter('K_ori', 15.0)
        self.declare_parameter('D_ori', 6.0)
        self.declare_parameter('control_rate', 100.0)
        
        K_pos = self.get_parameter('K_pos').value
        D_pos = self.get_parameter('D_pos').value
        K_ori = self.get_parameter('K_ori').value
        D_ori = self.get_parameter('D_ori').value
        
        self.K_d = np.diag([K_pos]*3 + [K_ori]*3)
        self.D_d = np.diag([D_pos]*3 + [D_ori]*3)
        
        # 양팔 상태
        self.arm_L = ArmState('L')
        self.arm_R = ArmState('R')
        
        # 위성 상태
        self.p_base = np.array([0.0, 0.0, 0.0])
        self.q_base = np.array([1.0, 0.0, 0.0, 0.0])
        self.v_base = np.zeros(3)
        self.omega_base = np.zeros(3)
        
        self.p_base_prev = self.p_base.copy()
        self.q_base_prev = self.q_base.copy()
        
        # Joint 이름 매핑
        self.joint_names_L = [f'arm_L_joint{i}' for i in range(1, 8)]
        self.joint_names_R = [f'arm_R_joint{i}' for i in range(1, 8)]
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        
        self.target_L_sub = self.create_subscription(
            PoseStamped, '/target_pose_L', self.target_L_callback, 10)
        
        self.target_R_sub = self.create_subscription(
            PoseStamped, '/target_pose_R', self.target_R_callback, 10)
        
        self.pose_sub = self.create_subscription(
            TFMessage, '/world/docking_world/pose/info', self.pose_callback, 10)
        
        # Publishers
        self.effort_L_pub = self.create_publisher(
            Float64MultiArray, '/effort_controller_L/commands', 10)
        self.effort_R_pub = self.create_publisher(
            Float64MultiArray, '/effort_controller_R/commands', 10)
        
        # 제어 타이머
        control_rate = self.get_parameter('control_rate').value
        self.dt = 1.0 / control_rate
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        # 플래그
        self.joint_received = False
        self.pose_received = False
        self.log_count = 0
        
        self.get_logger().info('Dual Arm Impedance Controller 시작')
        self.get_logger().info(f'K_pos={K_pos}, D_pos={D_pos}, K_ori={K_ori}, D_ori={D_ori}')
    
    def _quat_to_euler(self, q: np.ndarray) -> np.ndarray:
        """쿼터니언 → 오일러 (deg) [roll, pitch, yaw]"""
        R = quat_to_dcm(q)
        roll = np.arctan2(R[2,1], R[2,2]) * 180/np.pi
        pitch = -np.arcsin(np.clip(R[2,0], -1, 1)) * 180/np.pi
        yaw = np.arctan2(R[1,0], R[0,0]) * 180/np.pi
        return np.array([roll, pitch, yaw])
    
    def pose_callback(self, msg: TFMessage):
        """위성 pose 콜백"""
        for tf in msg.transforms:
            if 'satellite_body' in tf.child_frame_id or tf.child_frame_id == 'kari_dual_arm':
                t = tf.transform
                
                self.p_base_prev = self.p_base.copy()
                self.q_base_prev = self.q_base.copy()
                
                self.p_base = np.array([t.translation.x, t.translation.y, t.translation.z])
                self.q_base = np.array([t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z])
                
                self.v_base = (self.p_base - self.p_base_prev) / self.dt
                
                q_diff = quat_mult(self.q_base, quat_inv(self.q_base_prev))
                if q_diff[0] < 0:
                    q_diff = -q_diff
                self.omega_base = 2.0 * q_diff[1:4] / self.dt
                
                self.pose_received = True
                break
    
    def joint_callback(self, msg: JointState):
        """Joint state 콜백"""
        # Left arm
        for i, name in enumerate(self.joint_names_L):
            if name in msg.name:
                idx = msg.name.index(name)
                self.arm_L.theta[i] = msg.position[idx] if len(msg.position) > idx else 0.0
                self.arm_L.theta_dot[i] = msg.velocity[idx] if len(msg.velocity) > idx else 0.0
        
        # Right arm
        for i, name in enumerate(self.joint_names_R):
            if name in msg.name:
                idx = msg.name.index(name)
                self.arm_R.theta[i] = msg.position[idx] if len(msg.position) > idx else 0.0
                self.arm_R.theta_dot[i] = msg.velocity[idx] if len(msg.velocity) > idx else 0.0
        
        self.joint_received = True
        
        # 초기 타겟 설정
        if not self.arm_L.initialized and self.arm_L.target_position is None:
            fk_L = forward_kinematics(self.arm_L.params, self.p_base, self.q_base, self.arm_L.theta)
            self.arm_L.target_position = fk_L.p_ee.copy()
            self.arm_L.target_orientation = dcm_to_quat(fk_L.R_ee)
            self.arm_L.initialized = True
            q_L = self.arm_L.target_orientation
            euler_L = self._quat_to_euler(q_L)
            self.get_logger().info(f'Left EE 초기 pos: [{fk_L.p_ee[0]:.3f}, {fk_L.p_ee[1]:.3f}, {fk_L.p_ee[2]:.3f}]')
            self.get_logger().info(f'Left EE 초기 quat: [w={q_L[0]:.4f}, x={q_L[1]:.4f}, y={q_L[2]:.4f}, z={q_L[3]:.4f}]')
            self.get_logger().info(f'Left EE 초기 euler: [r={euler_L[0]:.2f}, p={euler_L[1]:.2f}, y={euler_L[2]:.2f}]deg')
        
        if not self.arm_R.initialized and self.arm_R.target_position is None:
            fk_R = forward_kinematics(self.arm_R.params, self.p_base, self.q_base, self.arm_R.theta)
            self.arm_R.target_position = fk_R.p_ee.copy()
            self.arm_R.target_orientation = dcm_to_quat(fk_R.R_ee)
            self.arm_R.initialized = True
            q_R = self.arm_R.target_orientation
            euler_R = self._quat_to_euler(q_R)
            self.get_logger().info(f'Right EE 초기 pos: [{fk_R.p_ee[0]:.3f}, {fk_R.p_ee[1]:.3f}, {fk_R.p_ee[2]:.3f}]')
            self.get_logger().info(f'Right EE 초기 quat: [w={q_R[0]:.4f}, x={q_R[1]:.4f}, y={q_R[2]:.4f}, z={q_R[3]:.4f}]')
            self.get_logger().info(f'Right EE 초기 euler: [r={euler_R[0]:.2f}, p={euler_R[1]:.2f}, y={euler_R[2]:.2f}]deg')
    
    def target_L_callback(self, msg: PoseStamped):
        """Left arm 목표 콜백"""
        self.arm_L.target_position = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.arm_L.target_orientation = np.array([
            msg.pose.orientation.w, msg.pose.orientation.x,
            msg.pose.orientation.y, msg.pose.orientation.z])
        self.get_logger().info(f'Left 새 목표: {self.arm_L.target_position}')
    
    def target_R_callback(self, msg: PoseStamped):
        """Right arm 목표 콜백"""
        self.arm_R.target_position = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.arm_R.target_orientation = np.array([
            msg.pose.orientation.w, msg.pose.orientation.x,
            msg.pose.orientation.y, msg.pose.orientation.z])
        self.get_logger().info(f'Right 새 목표: {self.arm_R.target_position}')
    
    def compute_arm_torque(self, arm: ArmState) -> np.ndarray:
        """단일 팔 임피던스 제어 토크 계산"""
        if arm.target_position is None:
            return np.zeros(7)
        
        # FK
        fk = forward_kinematics(arm.params, self.p_base, self.q_base, arm.theta)
        
        # Mass Matrix
        H = compute_mass_matrix(fk, arm.params)
        
        # Generalized Jacobian
        Jg = compute_generalized_jacobian(fk, H, arm.params)
        
        # Coriolis
        omega_inertial = quat_to_dcm(self.q_base) @ self.omega_base
        c = compute_coriolis(fk, arm.params, self.v_base, omega_inertial, arm.theta_dot)
        
        # Partition
        H_bb = H[0:6, 0:6]
        H_bm = H[0:6, 6:]
        H_mb = H[6:, 0:6]
        H_mm = H[6:, 6:]
        c_b = c[0:6]
        c_m = c[6:]
        
        # Effective dynamics
        H_bb_inv = np.linalg.inv(H_bb)
        H_eff = H_mm - H_mb @ H_bb_inv @ H_bm
        c_eff = c_m - H_mb @ H_bb_inv @ c_b
        
        # Task-space inertia
        H_eff_inv = np.linalg.inv(H_eff)
        Lambda_inv = Jg @ H_eff_inv @ Jg.T
        damping = 0.01
        Lambda = np.linalg.inv(Lambda_inv + damping * np.eye(6))
        
        mu = Lambda @ Jg @ H_eff_inv @ c_eff
        
        # EE 속도
        x_dot = Jg @ arm.theta_dot
        
        # 위치 오차
        e_pos = arm.target_position - fk.p_ee
        
        # 자세 오차
        q_ee = dcm_to_quat(fk.R_ee)
        q_ee_inv = quat_inv(q_ee)
        q_err = quat_mult(arm.target_orientation, q_ee_inv)
        if q_err[0] < 0:
            q_err = -q_err
        e_ori = 2.0 * q_err[1:4]
        
        e_x = np.concatenate([e_pos, e_ori])
        
        # 임피던스 제어
        x_ddot_cmd = self.K_d @ e_x - self.D_d @ x_dot
        F_imp = Lambda @ x_ddot_cmd + mu
        
        # Joint torque
        tau_cmd = Jg.T @ F_imp
        
        return tau_cmd, fk, e_pos
    
    def control_loop(self):
        """제어 루프"""
        if not self.joint_received:
            return
        
        # Left arm
        if self.arm_L.target_position is not None:
            tau_cmd_L, fk_L, e_pos_L = self.compute_arm_torque(self.arm_L)
            tau_dot_L, _ = motor_dynamics(self.arm_L.tau_actual, tau_cmd_L, self.arm_L.params)
            self.arm_L.tau_actual = self.arm_L.tau_actual + tau_dot_L * self.dt
        else:
            self.arm_L.tau_actual = np.zeros(7)
            fk_L = None
            e_pos_L = np.zeros(3)
        
        # Right arm
        if self.arm_R.target_position is not None:
            tau_cmd_R, fk_R, e_pos_R = self.compute_arm_torque(self.arm_R)
            tau_dot_R, _ = motor_dynamics(self.arm_R.tau_actual, tau_cmd_R, self.arm_R.params)
            self.arm_R.tau_actual = self.arm_R.tau_actual + tau_dot_R * self.dt
        else:
            self.arm_R.tau_actual = np.zeros(7)
            fk_R = None
            e_pos_R = np.zeros(3)
        
        # 토크 발행
        msg_L = Float64MultiArray()
        msg_L.data = self.arm_L.tau_actual.tolist()
        self.effort_L_pub.publish(msg_L)
        
        msg_R = Float64MultiArray()
        msg_R.data = self.arm_R.tau_actual.tolist()
        self.effort_R_pub.publish(msg_R)
        
        # 디버그 출력 (1초마다)
        self.log_count += 1
        if self.log_count % 100 == 0:
            pos_err_L = np.linalg.norm(e_pos_L) * 1000
            pos_err_R = np.linalg.norm(e_pos_R) * 1000
            max_tau_L = np.max(np.abs(self.arm_L.tau_actual))
            max_tau_R = np.max(np.abs(self.arm_R.tau_actual))
            
            # 위성 자세
            sat_euler = self._quat_to_euler(self.q_base)
            
            # EE 자세
            if fk_L is not None:
                ee_euler_L = self._quat_to_euler(dcm_to_quat(fk_L.R_ee))
                ee_pos_L = fk_L.p_ee
            else:
                ee_euler_L = np.zeros(3)
                ee_pos_L = np.zeros(3)
            
            if fk_R is not None:
                ee_euler_R = self._quat_to_euler(dcm_to_quat(fk_R.R_ee))
                ee_pos_R = fk_R.p_ee
            else:
                ee_euler_R = np.zeros(3)
                ee_pos_R = np.zeros(3)
            
            self.get_logger().info(
                f'=== Status ==='
            )
            self.get_logger().info(
                f'Sat pos: [{self.p_base[0]:.3f}, {self.p_base[1]:.3f}, {self.p_base[2]:.3f}], '
                f'att: [{sat_euler[0]:.2f}, {sat_euler[1]:.2f}, {sat_euler[2]:.2f}]deg'
            )
            self.get_logger().info(
                f'L: err={pos_err_L:.1f}mm, tau={max_tau_L:.1f}Nm, '
                f'ee=[{ee_pos_L[0]:.3f}, {ee_pos_L[1]:.3f}, {ee_pos_L[2]:.3f}], '
                f'att=[{ee_euler_L[0]:.1f}, {ee_euler_L[1]:.1f}, {ee_euler_L[2]:.1f}]'
            )
            self.get_logger().info(
                f'R: err={pos_err_R:.1f}mm, tau={max_tau_R:.1f}Nm, '
                f'ee=[{ee_pos_R[0]:.3f}, {ee_pos_R[1]:.3f}, {ee_pos_R[2]:.3f}], '
                f'att=[{ee_euler_R[0]:.1f}, {ee_euler_R[1]:.1f}, {ee_euler_R[2]:.1f}]'
            )


def main(args=None):
    rclpy.init(args=args)
    node = DualArmImpedanceController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        msg = Float64MultiArray()
        msg.data = [0.0] * 7
        node.effort_L_pub.publish(msg)
        node.effort_R_pub.publish(msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()