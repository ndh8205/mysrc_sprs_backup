#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import EntityWrench
from ros_gz_interfaces.srv import SetEntityPose
from geometry_msgs.msg import Wrench, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String

import tkinter as tk
from tkinter import ttk
import threading
import numpy as np
import math
import time

class GCOControllerNode(Node):
    """GCO 궤도 제어 및 수동 제어를 위한 ROS 2 노드"""
    
    def __init__(self, gui_root):
        super().__init__('gco_controller')
        
        # GUI root
        self.gui_root = gui_root
        
        # 제어 모드
        self.mode = "Manual"  # "Manual" or "GCO"
        self.gco_initialized = False
        
        # 궤도 파라미터 초기화
        self.initialize_orbit_params()
        
        # ROS 2 퍼블리셔 - EntityWrench (월드 이름 확인)
        self.wrench_publisher = self.create_publisher(
            EntityWrench,
            '/world/space_world/wrench',  # 실제 월드 이름으로 수정
            10
        )
        
        # 서비스 클라이언트 - SetEntityPose
        self.pose_client = self.create_client(
            SetEntityPose,
            '/world/space_world/set_pose'  # 실제 월드 이름으로 수정
        )
        
        # 구독자 - Odometry
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/model/nasa_satellite5/odometry',
            self.odom_callback,
            10
        )
        
        # 구독자 - IMU
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/nasa_satellite5/imu',
            self.imu_callback,
            10
        )
        
        # 현재 상태 저장
        self.current_state = {
            'position': np.zeros(3),
            'velocity': np.zeros(3),
            'orientation': np.zeros(4),  # quaternion
            'angular_velocity': np.zeros(3)
        }
        
        # 100Hz 제어 타이머 (0.01초)
        self.control_timer = self.create_timer(0.001, self.control_loop_callback)
        
        # GUI 변수들 - Manual 모드용
        self.force_x = tk.DoubleVar(value=0.0)
        self.force_y = tk.DoubleVar(value=0.0)
        self.force_z = tk.DoubleVar(value=0.0)
        self.torque_x = tk.DoubleVar(value=0.0)
        self.torque_y = tk.DoubleVar(value=0.0)
        self.torque_z = tk.DoubleVar(value=0.0)
        
        # GUI 변수들 - GCO 모드용
        self.phase_angle = tk.DoubleVar(value=0.0)
        self.orbit_radius = tk.DoubleVar(value=10.0)  # m (기본값 증가)
        
        # GUI 변수들 - 상태 표시
        self.pos_x_var = tk.StringVar(value="0.000")
        self.pos_y_var = tk.StringVar(value="0.000")
        self.pos_z_var = tk.StringVar(value="0.000")
        
        self.vel_x_var = tk.StringVar(value="0.000")
        self.vel_y_var = tk.StringVar(value="0.000")
        self.vel_z_var = tk.StringVar(value="0.000")
        
        self.roll_var = tk.StringVar(value="0.0")
        self.pitch_var = tk.StringVar(value="0.0")
        self.yaw_var = tk.StringVar(value="0.0")
        
        self.ang_vel_x_var = tk.StringVar(value="0.000")
        self.ang_vel_y_var = tk.StringVar(value="0.000")
        self.ang_vel_z_var = tk.StringVar(value="0.000")
        
        # 제어력 표시 변수 추가
        self.control_fx_var = tk.StringVar(value="0.000")
        self.control_fy_var = tk.StringVar(value="0.000")
        self.control_fz_var = tk.StringVar(value="0.000")
        
        # GUI 설정
        self.setup_gui()
        
        self.get_logger().info("GCO Controller Node Started")
        
    def initialize_orbit_params(self):
        """궤도 파라미터 초기화 (SI 단위)"""
        self.params = {}
        
        # 기본 상수
        self.params['mu'] = 398600.441799999971e9  # m³/s² (km³/s² → m³/s²)
        self.params['J2'] = 0.0010827
        self.params['R_e'] = 6378137.0  # m (km → m)
        
        # 참조 궤도
        self.params['a_ref'] = 6892137.0  # m (km → m)
        self.params['i_ref'] = np.deg2rad(97.45)  # rad
        
        # Deputy 위성 질량 (가정값, 실제 모델에 맞게 조정 필요)
        self.params['m_deputy'] = 1  # kg
        
        # 평균 운동
        self.params['n'] = np.sqrt(self.params['mu'] / self.params['a_ref']**3)
        
        # J2 섭동 계수
        r_ref = self.params['a_ref']
        i_ref = self.params['i_ref']
        self.params['s'] = (3 * self.params['J2'] * self.params['R_e']**2) / \
                          (8 * r_ref**2) * (1 + 3 * np.cos(2 * i_ref))
        self.params['c'] = np.sqrt(1 + self.params['s'])
        
        self.get_logger().info(f"Orbit parameters initialized: n={self.params['n']:.6e}, c={self.params['c']:.6f}")
        
    def setup_gui(self):
        """GUI 설정"""
        self.gui_root.title("GCO & Manual Satellite Control (ROS 2)")
        
        # 메인 컨테이너
        main_container = ttk.Frame(self.gui_root, padding="10")
        main_container.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 왼쪽: 제어 패널
        control_frame = ttk.Frame(main_container)
        control_frame.grid(row=0, column=0, padx=5, sticky="nsew")
        
        # 오른쪽: 상태 모니터링 패널
        monitor_frame = ttk.Frame(main_container)
        monitor_frame.grid(row=0, column=1, padx=5, sticky="nsew")
        
        # --- 제어 패널 구성 ---
        ttk.Label(control_frame, text="CONTROL PANEL", font=("Arial", 12, "bold")).grid(row=0, column=0, pady=5)
        
        # 모드 선택
        mode_frame = ttk.LabelFrame(control_frame, text="Control Mode", padding="10")
        mode_frame.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        
        self.mode_var = tk.StringVar(value="Manual")
        ttk.Radiobutton(mode_frame, text="Manual Mode", variable=self.mode_var, 
                       value="Manual", command=self.on_mode_change).grid(row=0, column=0, sticky="w")
        ttk.Radiobutton(mode_frame, text="GCO Mode", variable=self.mode_var, 
                       value="GCO", command=self.on_mode_change).grid(row=0, column=1, sticky="w")
        
        # GCO 파라미터
        self.gco_frame = ttk.LabelFrame(control_frame, text="GCO Parameters", padding="10")
        self.gco_frame.grid(row=2, column=0, padx=5, pady=5, sticky="ew")
        
        # Phase Angle
        ttk.Label(self.gco_frame, text="Phase Angle (deg):").grid(row=0, column=0, sticky="w")
        self.phase_slider = ttk.Scale(self.gco_frame, from_=0, to=360, orient="horizontal", 
                                     variable=self.phase_angle, length=200)
        self.phase_slider.grid(row=0, column=1)
        self.phase_label = ttk.Label(self.gco_frame, textvariable=self.phase_angle, width=7)
        self.phase_label.grid(row=0, column=2)
        
        # Orbit Radius
        ttk.Label(self.gco_frame, text="Orbit Radius (m):").grid(row=1, column=0, sticky="w")
        self.radius_slider = ttk.Scale(self.gco_frame, from_=1.0, to=100.0, orient="horizontal", 
                                      variable=self.orbit_radius, length=200)
        self.radius_slider.grid(row=1, column=1)
        self.radius_label = ttk.Label(self.gco_frame, text="10.000", width=7)
        self.radius_label.grid(row=1, column=2)
        
        # Initialize GCO Button
        self.init_gco_btn = ttk.Button(self.gco_frame, text="Initialize GCO", 
                                      command=self.initialize_gco)
        self.init_gco_btn.grid(row=2, column=0, columnspan=3, pady=10)
        
        # Manual Force Controls
        self.manual_frame = ttk.Frame(control_frame)
        self.manual_frame.grid(row=3, column=0, sticky="ew")
        
        # Force controls (더 작은 범위로 조정)
        force_frame = ttk.LabelFrame(self.manual_frame, text="Force (N)", padding="10")
        force_frame.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        self.create_slider_set(force_frame, "Force", self.force_x, self.force_y, self.force_z, -1.0, 1.0)
        
        # Torque controls (더 작은 범위로 조정)
        torque_frame = ttk.LabelFrame(self.manual_frame, text="Torque (N·m)", padding="10")
        torque_frame.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        self.create_slider_set(torque_frame, "Torque", self.torque_x, self.torque_y, self.torque_z, -0.1, 0.1)
        
        # Control buttons
        button_frame = ttk.Frame(control_frame, padding="10")
        button_frame.grid(row=4, column=0, sticky="ew")
        ttk.Button(button_frame, text="Reset All", command=self.reset_values).pack(side="left", padx=5)
        ttk.Button(button_frame, text="Stop GCO", command=self.stop_gco).pack(side="left", padx=5)
        ttk.Button(button_frame, text="Quit", command=self.shutdown).pack(side="right", padx=5)
        
        # --- 모니터링 패널 구성 ---
        ttk.Label(monitor_frame, text="STATE MONITORING", font=("Arial", 12, "bold")).grid(row=0, column=0, pady=5)
        
        # Position
        position_frame = ttk.LabelFrame(monitor_frame, text="Position (m)", padding="10")
        position_frame.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        self.create_display_set(position_frame, ["X:", "Y:", "Z:"], 
                               [self.pos_x_var, self.pos_y_var, self.pos_z_var])
        
        # Velocity
        velocity_frame = ttk.LabelFrame(monitor_frame, text="Linear Velocity (m/s)", padding="10")
        velocity_frame.grid(row=2, column=0, padx=5, pady=5, sticky="ew")
        self.create_display_set(velocity_frame, ["Vx:", "Vy:", "Vz:"], 
                               [self.vel_x_var, self.vel_y_var, self.vel_z_var])
        
        # Attitude
        attitude_frame = ttk.LabelFrame(monitor_frame, text="Attitude (deg)", padding="10")
        attitude_frame.grid(row=3, column=0, padx=5, pady=5, sticky="ew")
        self.create_display_set(attitude_frame, ["Roll:", "Pitch:", "Yaw:"], 
                               [self.roll_var, self.pitch_var, self.yaw_var])
        
        # Angular Velocity
        ang_velocity_frame = ttk.LabelFrame(monitor_frame, text="Angular Velocity (rad/s)", padding="10")
        ang_velocity_frame.grid(row=4, column=0, padx=5, pady=5, sticky="ew")
        self.create_display_set(ang_velocity_frame, ["ωx:", "ωy:", "ωz:"], 
                               [self.ang_vel_x_var, self.ang_vel_y_var, self.ang_vel_z_var])
        
        # Control Forces (GCO 모드일 때 표시)
        self.control_force_frame = ttk.LabelFrame(monitor_frame, text="Control Forces (N)", padding="10")
        self.control_force_frame.grid(row=5, column=0, padx=5, pady=5, sticky="ew")
        self.create_display_set(self.control_force_frame, ["Fx:", "Fy:", "Fz:"],
                               [self.control_fx_var, self.control_fy_var, self.control_fz_var])
        
        # Update slider labels
        self.phase_slider.bind("<Motion>", lambda e: self.phase_label.config(text=f"{self.phase_angle.get():.1f}"))
        self.radius_slider.bind("<Motion>", lambda e: self.radius_label.config(text=f"{self.orbit_radius.get():.3f}"))
        
        # 초기 모드 설정
        self.on_mode_change()
        
    def create_slider_set(self, parent, name, var_x, var_y, var_z, from_, to):
        """슬라이더 3개 (X, Y, Z) 세트를 생성"""
        resolution = 0.001 if abs(to - from_) <= 2 else 0.01  # 작은 범위에서는 더 높은 정밀도
        
        ttk.Label(parent, text="X:").grid(row=0, column=0, sticky="w")
        scale_x = ttk.Scale(parent, from_=from_, to=to, orient="horizontal", variable=var_x, length=200)
        scale_x.grid(row=0, column=1)
        ttk.Label(parent, textvariable=var_x, width=7).grid(row=0, column=2)

        ttk.Label(parent, text="Y:").grid(row=1, column=0, sticky="w")
        scale_y = ttk.Scale(parent, from_=from_, to=to, orient="horizontal", variable=var_y, length=200)
        scale_y.grid(row=1, column=1)
        ttk.Label(parent, textvariable=var_y, width=7).grid(row=1, column=2)

        ttk.Label(parent, text="Z:").grid(row=2, column=0, sticky="w")
        scale_z = ttk.Scale(parent, from_=from_, to=to, orient="horizontal", variable=var_z, length=200)
        scale_z.grid(row=2, column=1)
        ttk.Label(parent, textvariable=var_z, width=7).grid(row=2, column=2)
        
    def create_display_set(self, parent, labels, variables):
        """상태 표시용 라벨 세트를 생성"""
        for i, (label, var) in enumerate(zip(labels, variables)):
            ttk.Label(parent, text=label, width=6).grid(row=i, column=0, sticky="w", padx=5)
            display_label = ttk.Label(parent, textvariable=var, width=12, 
                                    relief="sunken", anchor="e", 
                                    font=("Courier", 10))
            display_label.grid(row=i, column=1, sticky="ew", padx=5, pady=2)
    
    def on_mode_change(self):
        """제어 모드 변경 시 호출"""
        self.mode = self.mode_var.get()
        
        if self.mode == "Manual":
            # Manual 모드: GCO 파라미터 비활성화, Manual 컨트롤 활성화
            for child in self.gco_frame.winfo_children():
                if isinstance(child, (ttk.Scale, ttk.Button)):
                    child.configure(state="disabled")
            for child in self.manual_frame.winfo_children():
                for widget in child.winfo_children():
                    if isinstance(widget, ttk.Scale):
                        widget.configure(state="normal")
            self.gco_initialized = False
            self.get_logger().info("Switched to Manual mode")
            
        else:  # GCO mode
            # GCO 모드: GCO 파라미터 활성화, Manual 컨트롤 비활성화
            for child in self.gco_frame.winfo_children():
                if isinstance(child, (ttk.Scale, ttk.Button)):
                    child.configure(state="normal")
            for child in self.manual_frame.winfo_children():
                for widget in child.winfo_children():
                    if isinstance(widget, ttk.Scale):
                        widget.configure(state="disabled")
            self.get_logger().info("Switched to GCO mode - Please initialize GCO")
    
    def initialize_gco(self):
        """GCO 궤도 초기화"""
        phase_deg = self.phase_angle.get()
        radius_m = self.orbit_radius.get()
        
        self.get_logger().info(f"Initializing GCO: phase={phase_deg:.1f}°, radius={radius_m:.3f}m")
        
        # 초기 상태 계산
        phase_rad = np.deg2rad(phase_deg)
        n = self.params['n']
        
        # GCO 초기 조건 (LVLH 좌표계)
        x_0 = radius_m / 2 * np.sin(phase_rad)
        vx_0 = radius_m * n / 2 * np.cos(phase_rad)
        y_0 = 2 * vx_0 / n
        vy_0 = -2 * n * x_0
        z_0 = np.sqrt(3) * x_0
        vz_0 = np.sqrt(3) * vx_0
        
        # Deputy 위성 위치 설정 (SetEntityPose 서비스 사용)
        if not self.pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("SetEntityPose service not available")
            return
        
        request = SetEntityPose.Request()
        request.entity.name = 'deputy_satellite'
        request.entity.type = 2  # MODEL
        
        pose = Pose()
        # numpy float를 Python float로 변환
        pose.position.x = float(x_0)
        pose.position.y = float(y_0)
        pose.position.z = float(z_0)
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        
        request.pose = pose
        
        # 비동기 서비스 호출
        future = self.pose_client.call_async(request)
        future.add_done_callback(self.pose_service_callback)
        
        # 초기 속도 저장 (제어에 사용)
        self.initial_velocity = np.array([vx_0, vy_0, vz_0])
        self.gco_initialized = True
        
        self.get_logger().info(f"GCO Initial State - Pos: [{x_0:.3f}, {y_0:.3f}, {z_0:.3f}] m")
        self.get_logger().info(f"GCO Initial State - Vel: [{vx_0:.3f}, {vy_0:.3f}, {vz_0:.3f}] m/s")
    
    def pose_service_callback(self, future):
        """SetEntityPose 서비스 응답 콜백"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Successfully set initial GCO position")
            else:
                self.get_logger().error("Failed to set initial GCO position")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")
    
    def stop_gco(self):
        """GCO 제어 중지"""
        self.gco_initialized = False
        self.mode_var.set("Manual")
        self.on_mode_change()
        self.reset_values()
        self.get_logger().info("GCO control stopped")
    
    def control_loop_callback(self):
        """100Hz 제어 루프"""
        if self.mode == "GCO" and self.gco_initialized:
            # GCO 제어력 계산
            x, y, z = self.current_state['position']
            vx, vy, vz = self.current_state['velocity']
            
            c = self.params['c']
            n = self.params['n']
            m = self.params['m_deputy']
            
            # GCO 유지를 위한 제어력 (J2 섭동 포함)
            F_x = m * ((5*c**2 - 2)*n**2*x + 2*n*c*vy)
            F_y = -m * 2*n*c*vx
            F_z = -m * (3*c**2 - 2)*n**2*z
            
            # numpy float64를 Python float로 변환
            F_x = float(F_x)
            F_y = float(F_y)
            F_z = float(F_z)
            
            # 제어력 표시 업데이트
            self.control_fx_var.set(f"{F_x:.3f}")
            self.control_fy_var.set(f"{F_y:.3f}")
            self.control_fz_var.set(f"{F_z:.3f}")
            
            # 주기적으로 제어력 로그 출력 (디버깅용)
            if hasattr(self, '_control_counter'):
                self._control_counter += 1
            else:
                self._control_counter = 0
                
            if self._control_counter % 5000 == 0:  # 1초마다
                self.get_logger().info(f"GCO Control - Pos:[{x:.3f},{y:.3f},{z:.3f}] Force:[{F_x:.3f},{F_y:.3f},{F_z:.3f}]N")
            
            # EntityWrench 발행
            self.publish_wrench(F_x, F_y, F_z, 0.0, 0.0, 0.0)
            
        elif self.mode == "Manual":
            # Manual 모드: GUI 슬라이더 값 사용
            F_x = float(self.force_x.get())
            F_y = float(self.force_y.get())
            F_z = float(self.force_z.get())
            T_x = float(self.torque_x.get())
            T_y = float(self.torque_y.get())
            T_z = float(self.torque_z.get())
            
            self.publish_wrench(F_x, F_y, F_z, T_x, T_y, T_z)
    
    def publish_wrench(self, fx, fy, fz, tx, ty, tz):
        """EntityWrench 메시지 발행"""
        msg = EntityWrench()
        
        # Entity 정보 - 링크에 직접 적용
        msg.entity.name = 'deputy_satellite::nasa_satellite_link5'  # 모델명::링크명
        msg.entity.type = 3  # LINK (not MODEL)
        
        # Wrench 정보 - 모든 값을 Python float로 변환
        msg.wrench.force = Vector3(x=float(fx), y=float(fy), z=float(fz))
        msg.wrench.torque = Vector3(x=float(tx), y=float(ty), z=float(tz))
        
        self.wrench_publisher.publish(msg)
    
    def odom_callback(self, msg):
        """Odometry 메시지 콜백"""
        # 위치 업데이트
        self.current_state['position'][0] = msg.pose.pose.position.x
        self.current_state['position'][1] = msg.pose.pose.position.y
        self.current_state['position'][2] = msg.pose.pose.position.z
        
        # 속도 업데이트
        self.current_state['velocity'][0] = msg.twist.twist.linear.x
        self.current_state['velocity'][1] = msg.twist.twist.linear.y
        self.current_state['velocity'][2] = msg.twist.twist.linear.z
        
        # GUI 업데이트
        self.pos_x_var.set(f"{self.current_state['position'][0]:.3f}")
        self.pos_y_var.set(f"{self.current_state['position'][1]:.3f}")
        self.pos_z_var.set(f"{self.current_state['position'][2]:.3f}")
        
        self.vel_x_var.set(f"{self.current_state['velocity'][0]:.3f}")
        self.vel_y_var.set(f"{self.current_state['velocity'][1]:.3f}")
        self.vel_z_var.set(f"{self.current_state['velocity'][2]:.3f}")
    
    def imu_callback(self, msg):
        """IMU 메시지 콜백"""
        # 쿼터니언을 오일러 각으로 변환
        q = msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
        
        # 자세 업데이트 (라디안을 도로 변환)
        self.roll_var.set(f"{math.degrees(roll):.1f}")
        self.pitch_var.set(f"{math.degrees(pitch):.1f}")
        self.yaw_var.set(f"{math.degrees(yaw):.1f}")
        
        # 각속도 업데이트
        self.current_state['angular_velocity'][0] = msg.angular_velocity.x
        self.current_state['angular_velocity'][1] = msg.angular_velocity.y
        self.current_state['angular_velocity'][2] = msg.angular_velocity.z
        
        self.ang_vel_x_var.set(f"{msg.angular_velocity.x:.3f}")
        self.ang_vel_y_var.set(f"{msg.angular_velocity.y:.3f}")
        self.ang_vel_z_var.set(f"{msg.angular_velocity.z:.3f}")
    
    def quaternion_to_euler(self, x, y, z, w):
        """쿼터니언을 오일러 각으로 변환"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def reset_values(self):
        """모든 슬라이더 값을 0으로 리셋"""
        self.force_x.set(0.0)
        self.force_y.set(0.0)
        self.force_z.set(0.0)
        self.torque_x.set(0.0)
        self.torque_y.set(0.0)
        self.torque_z.set(0.0)
        
        # 제어력 표시도 리셋
        self.control_fx_var.set("0.000")
        self.control_fy_var.set("0.000")
        self.control_fz_var.set("0.000")
    
    def shutdown(self):
        """노드와 GUI를 안전하게 종료"""
        self.get_logger().info('Shutting down GCO Controller node.')
        self.destroy_node()
        self.gui_root.quit()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    # GUI와 ROS 노드 생성
    root = tk.Tk()
    node = GCOControllerNode(root)
    
    # ROS 노드를 별도 스레드에서 실행
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    # 메인 스레드에서 GUI 실행
    root.mainloop()


if __name__ == '__main__':
    main()