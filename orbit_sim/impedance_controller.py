#!/usr/bin/env python3
"""
impedance_controller.py
Task-space Impedance Controller for KARI ARM (Free-floating)

Subscribes:
    /joint_states (sensor_msgs/JointState)
    /target_pose (geometry_msgs/PoseStamped) - 목표 EE 포즈
    /world/kari_arm_world/pose/info (tf2_msgs/TFMessage) - 위성 pose

Publishes:
    /effort_controller/commands (std_msgs/Float64MultiArray)

Author: Space Challenge Project
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage
import numpy as np

# 동역학 라이브러리
from orbit_sim.kari_arm_dynamics import (
    KariArmParams, forward_kinematics, compute_mass_matrix,
    compute_generalized_jacobian, compute_coriolis,
    motor_dynamics, quat_to_dcm, dcm_to_quat, quat_mult, quat_inv
)


class ImpedanceController(Node):
    def __init__(self):
        super().__init__('impedance_controller')
        
        # 파라미터
        self.params = KariArmParams()
        
        # 임피던스 게인
        self.declare_parameter('K_pos', 50.0)   # 위치 강성 [N/m]
        self.declare_parameter('D_pos', 20.0)   # 위치 감쇠 [Ns/m]
        self.declare_parameter('K_ori', 15.0)   # 자세 강성 [Nm/rad]
        self.declare_parameter('D_ori', 6.0)    # 자세 감쇠 [Nms/rad]
        self.declare_parameter('control_rate', 100.0)  # Hz
        
        K_pos = self.get_parameter('K_pos').value
        D_pos = self.get_parameter('D_pos').value
        K_ori = self.get_parameter('K_ori').value
        D_ori = self.get_parameter('D_ori').value
        
        self.K_d = np.diag([K_pos]*3 + [K_ori]*3)
        self.D_d = np.diag([D_pos]*3 + [D_ori]*3)
        
        # 상태 변수
        self.theta = np.zeros(7)
        self.theta_dot = np.zeros(7)
        self.tau_actual = np.zeros(7)
        
        # 위성 상태 (Free-floating) - 실시간 업데이트됨
        self.p_base = np.array([0.0, 0.0, 2.0])
        self.q_base = np.array([1.0, 0.0, 0.0, 0.0])
        self.v_base = np.zeros(3)
        self.omega_base = np.zeros(3)
        
        # 이전 위성 상태 (속도 계산용)
        self.p_base_prev = self.p_base.copy()
        self.q_base_prev = self.q_base.copy()
        
        # 목표 포즈
        self.target_position = None
        self.target_orientation = None  # quaternion [w,x,y,z]
        
        # Joint 이름 매핑
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 
                           'joint5', 'joint6', 'joint7']
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        
        self.target_sub = self.create_subscription(
            PoseStamped, '/target_pose', self.target_callback, 10)
        
        self.pose_sub = self.create_subscription(
            TFMessage, '/world/kari_arm_world/pose/info', self.pose_callback, 10)
        
        # Publisher
        self.effort_pub = self.create_publisher(
            Float64MultiArray, '/effort_controller/commands', 10)
        
        # 제어 타이머
        control_rate = self.get_parameter('control_rate').value
        self.dt = 1.0 / control_rate
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        # 초기화 플래그
        self.initialized = False
        self.joint_received = False
        self.pose_received = False
        
        self.get_logger().info('Impedance Controller 시작')
        self.get_logger().info(f'K_pos={K_pos}, D_pos={D_pos}, K_ori={K_ori}, D_ori={D_ori}')
    
    def pose_callback(self, msg: TFMessage):
        """위성 pose 콜백 (transforms[0] = satellite_body)"""
        if len(msg.transforms) > 0:
            t = msg.transforms[0].transform
            
            # 이전 값 저장 (속도 계산용)
            self.p_base_prev = self.p_base.copy()
            self.q_base_prev = self.q_base.copy()
            
            # 현재 값 업데이트
            self.p_base = np.array([t.translation.x, t.translation.y, t.translation.z])
            self.q_base = np.array([t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z])
            
            # 속도 추정 (수치 미분)
            self.v_base = (self.p_base - self.p_base_prev) / self.dt
            
            # 각속도 추정 (쿼터니언 미분으로부터)
            q_diff = quat_mult(self.q_base, quat_inv(self.q_base_prev))
            if q_diff[0] < 0:
                q_diff = -q_diff
            self.omega_base = 2.0 * q_diff[1:4] / self.dt
            
            self.pose_received = True
    
    def joint_callback(self, msg: JointState):
        """Joint state 콜백"""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.theta[i] = msg.position[idx] if len(msg.position) > idx else 0.0
                self.theta_dot[i] = msg.velocity[idx] if len(msg.velocity) > idx else 0.0
        
        self.joint_received = True
        
        # 초기 타겟 설정 (현재 EE 위치)
        if not self.initialized and self.target_position is None:
            fk = forward_kinematics(self.params, self.p_base, self.q_base, self.theta)
            self.target_position = fk.p_ee.copy()
            self.target_orientation = dcm_to_quat(fk.R_ee)
            self.initialized = True
            self.get_logger().info(f'초기 EE 위치: {self.target_position}')
            self.get_logger().info(f'초기 위성 위치: {self.p_base}')
    
    def target_callback(self, msg: PoseStamped):
        """목표 포즈 콜백"""
        self.target_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        self.target_orientation = np.array([
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z
        ])
        self.get_logger().info(f'새 목표: pos={self.target_position}')
    
    def control_loop(self):
        """제어 루프"""
        if not self.joint_received or self.target_position is None:
            return
        
        # FK (실시간 위성 pose 사용)
        fk = forward_kinematics(self.params, self.p_base, self.q_base, self.theta)
        
        # Mass Matrix
        H = compute_mass_matrix(fk, self.params)
        
        # Generalized Jacobian
        Jg = compute_generalized_jacobian(fk, H, self.params)
        
        # Coriolis
        omega_inertial = quat_to_dcm(self.q_base) @ self.omega_base
        c = compute_coriolis(fk, self.params, self.v_base, omega_inertial, self.theta_dot)
        
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
        x_dot = Jg @ self.theta_dot
        
        # Task-space 오차
        e_pos = self.target_position - fk.p_ee
        
        # 자세 오차 (quaternion)
        q_ee = dcm_to_quat(fk.R_ee)
        q_ee_inv = quat_inv(q_ee)
        q_err = quat_mult(self.target_orientation, q_ee_inv)
        if q_err[0] < 0:
            q_err = -q_err
        e_ori = 2.0 * q_err[1:4]
        
        e_x = np.concatenate([e_pos, e_ori])
        
        # 임피던스 제어
        x_ddot_cmd = self.K_d @ e_x - self.D_d @ x_dot
        F_imp = Lambda @ x_ddot_cmd + mu
        
        # Joint torque
        tau_cmd = Jg.T @ F_imp
        
        # 모터 다이나믹스
        tau_dot, tau_sat = motor_dynamics(self.tau_actual, tau_cmd, self.params)
        self.tau_actual = self.tau_actual + tau_dot * self.dt
        
        # 토크 발행
        msg = Float64MultiArray()
        msg.data = self.tau_actual.tolist()
        self.effort_pub.publish(msg)
        
        # 디버그 출력 (1초마다)
        if hasattr(self, 'log_count'):
            self.log_count += 1
        else:
            self.log_count = 0
        
        if self.log_count % 100 == 0:
            pos_err = np.linalg.norm(e_pos) * 1000  # mm
            max_tau = np.max(np.abs(self.tau_actual))
            
            # 위성 자세 (quaternion → euler in degrees)
            R_sat = quat_to_dcm(self.q_base)
            roll = np.arctan2(R_sat[2,1], R_sat[2,2]) * 180/np.pi
            pitch = -np.arcsin(np.clip(R_sat[2,0], -1, 1)) * 180/np.pi
            yaw = np.arctan2(R_sat[1,0], R_sat[0,0]) * 180/np.pi
            
            self.get_logger().info(
                f'pos_err={pos_err:.1f}mm, max_tau={max_tau:.1f}Nm, '
                f'ee=[{fk.p_ee[0]:.3f}, {fk.p_ee[1]:.3f}, {fk.p_ee[2]:.3f}], '
                f'sat_pos=[{self.p_base[0]:.3f}, {self.p_base[1]:.3f}, {self.p_base[2]:.3f}], '
                f'sat_att=[{roll:.2f}, {pitch:.2f}, {yaw:.2f}]deg'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ImpedanceController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 토크 0으로 설정
        msg = Float64MultiArray()
        msg.data = [0.0] * 7
        node.effort_pub.publish(msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()