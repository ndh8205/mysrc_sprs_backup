#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import EntityWrench
from geometry_msgs.msg import Wrench, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String

import tkinter as tk
from tkinter import ttk
import threading
import math

class ForceTorqueControllerNode(Node):
    """GUI를 통해 받은 값으로 EntityWrench 메시지를 발행하고 상태를 모니터링하는 ROS 2 노드"""
    def __init__(self, gui_root):
        super().__init__('force_torque_controller')
        
        # ROS 2 퍼블리셔 생성 - EntityWrench 타입
        self.publisher_ = self.create_publisher(
            EntityWrench, 
            '/world/experiment_world/wrench',
            10
        )
        
        # 상태 정보 구독자 생성
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/model/nasa_satellite5/odometry',  # 수정됨
            self.odom_callback,
            10
        )
        
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/nasa_satellite5/imu',  # 수정됨
            self.imu_callback,
            10
        )
        
        # 0.1초마다 (10Hz) publish_wrench_callback 함수를 호출하는 타이머 생성
        self.timer = self.create_timer(0.1, self.publish_wrench_callback)
        
        self.gui_root = gui_root
        
        # GUI를 위한 변수들 - 제어 입력
        self.force_x = tk.DoubleVar()
        self.force_y = tk.DoubleVar()
        self.force_z = tk.DoubleVar()
        self.torque_x = tk.DoubleVar()
        self.torque_y = tk.DoubleVar()
        self.torque_z = tk.DoubleVar()
        
        # GUI를 위한 변수들 - 상태 표시
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

        self.setup_gui()

    def setup_gui(self):
        """Tkinter GUI를 설정하는 메소드"""
        self.gui_root.title("Satellite Control & Monitoring (ROS 2)")
        
        # 메인 프레임을 좌우로 나누기
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
        
        # 힘 제어 UI
        force_frame = ttk.LabelFrame(control_frame, text="Force (N)", padding="10")
        force_frame.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        self.create_slider_set(force_frame, "Force", self.force_x, self.force_y, self.force_z, -50, 50)

        # 토크 제어 UI
        torque_frame = ttk.LabelFrame(control_frame, text="Torque (N·m)", padding="10")
        torque_frame.grid(row=2, column=0, padx=5, pady=5, sticky="ew")
        self.create_slider_set(torque_frame, "Torque", self.torque_x, self.torque_y, self.torque_z, -10, 10)

        # 제어 버튼
        button_frame = ttk.Frame(control_frame, padding="10")
        button_frame.grid(row=3, column=0, sticky="ew")
        ttk.Button(button_frame, text="Reset All", command=self.reset_values).pack(side="left", padx=5)
        ttk.Button(button_frame, text="Quit", command=self.shutdown).pack(side="right", padx=5)
        
        # --- 모니터링 패널 구성 ---
        ttk.Label(monitor_frame, text="STATE MONITORING", font=("Arial", 12, "bold")).grid(row=0, column=0, pady=5)
        
        # 위치 정보
        position_frame = ttk.LabelFrame(monitor_frame, text="Position (m)", padding="10")
        position_frame.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        self.create_display_set(position_frame, ["X:", "Y:", "Z:"], 
                               [self.pos_x_var, self.pos_y_var, self.pos_z_var])
        
        # 속도 정보
        velocity_frame = ttk.LabelFrame(monitor_frame, text="Linear Velocity (m/s)", padding="10")
        velocity_frame.grid(row=2, column=0, padx=5, pady=5, sticky="ew")
        self.create_display_set(velocity_frame, ["Vx:", "Vy:", "Vz:"], 
                               [self.vel_x_var, self.vel_y_var, self.vel_z_var])
        
        # 자세 정보 (오일러 각)
        attitude_frame = ttk.LabelFrame(monitor_frame, text="Attitude (deg)", padding="10")
        attitude_frame.grid(row=3, column=0, padx=5, pady=5, sticky="ew")
        self.create_display_set(attitude_frame, ["Roll:", "Pitch:", "Yaw:"], 
                               [self.roll_var, self.pitch_var, self.yaw_var])
        
        # 각속도 정보
        ang_velocity_frame = ttk.LabelFrame(monitor_frame, text="Angular Velocity (rad/s)", padding="10")
        ang_velocity_frame.grid(row=4, column=0, padx=5, pady=5, sticky="ew")
        self.create_display_set(ang_velocity_frame, ["ωx:", "ωy:", "ωz:"], 
                               [self.ang_vel_x_var, self.ang_vel_y_var, self.ang_vel_z_var])
        
    def create_slider_set(self, parent, name, var_x, var_y, var_z, from_, to):
        """슬라이더 3개 (X, Y, Z) 세트를 생성"""
        ttk.Label(parent, text=f"X:").grid(row=0, column=0, sticky="w")
        ttk.Scale(parent, from_=from_, to=to, orient="horizontal", variable=var_x, length=200).grid(row=0, column=1)
        ttk.Label(parent, textvariable=var_x, width=7).grid(row=0, column=2)

        ttk.Label(parent, text=f"Y:").grid(row=1, column=0, sticky="w")
        ttk.Scale(parent, from_=from_, to=to, orient="horizontal", variable=var_y, length=200).grid(row=1, column=1)
        ttk.Label(parent, textvariable=var_y, width=7).grid(row=1, column=2)

        ttk.Label(parent, text=f"Z:").grid(row=2, column=0, sticky="w")
        ttk.Scale(parent, from_=from_, to=to, orient="horizontal", variable=var_z, length=200).grid(row=2, column=1)
        ttk.Label(parent, textvariable=var_z, width=7).grid(row=2, column=2)
    
    def create_display_set(self, parent, labels, variables):
        """상태 표시용 라벨 세트를 생성"""
        for i, (label, var) in enumerate(zip(labels, variables)):
            ttk.Label(parent, text=label, width=6).grid(row=i, column=0, sticky="w", padx=5)
            display_label = ttk.Label(parent, textvariable=var, width=12, 
                                    relief="sunken", anchor="e", 
                                    font=("Courier", 10))
            display_label.grid(row=i, column=1, sticky="ew", padx=5, pady=2)
        
    def publish_wrench_callback(self):
        """타이머에 의해 주기적으로 호출되어 EntityWrench 메시지를 발행"""
        msg = EntityWrench()
        
        # Entity 정보 설정
        msg.entity.name = 'nasa_satellite5'  # 수정됨
        msg.entity.type = 2  # MODEL 타입
        
        # Wrench 정보 설정
        msg.wrench.force = Vector3(x=self.force_x.get(), y=self.force_y.get(), z=self.force_z.get())
        msg.wrench.torque = Vector3(x=self.torque_x.get(), y=self.torque_y.get(), z=self.torque_z.get())
        
        self.publisher_.publish(msg)
    
    def odom_callback(self, msg):
        """Odometry 메시지 콜백 - 위치와 속도 정보 업데이트"""
        # 위치 업데이트
        self.pos_x_var.set(f"{msg.pose.pose.position.x:.3f}")
        self.pos_y_var.set(f"{msg.pose.pose.position.y:.3f}")
        self.pos_z_var.set(f"{msg.pose.pose.position.z:.3f}")
        
        # 선속도 업데이트
        self.vel_x_var.set(f"{msg.twist.twist.linear.x:.3f}")
        self.vel_y_var.set(f"{msg.twist.twist.linear.y:.3f}")
        self.vel_z_var.set(f"{msg.twist.twist.linear.z:.3f}")
    
    def imu_callback(self, msg):
        """IMU 메시지 콜백 - 자세와 각속도 정보 업데이트"""
        # 쿼터니언을 오일러 각으로 변환
        q = msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
        
        # 자세 업데이트 (라디안을 도로 변환)
        self.roll_var.set(f"{math.degrees(roll):.1f}")
        self.pitch_var.set(f"{math.degrees(pitch):.1f}")
        self.yaw_var.set(f"{math.degrees(yaw):.1f}")
        
        # 각속도 업데이트
        self.ang_vel_x_var.set(f"{msg.angular_velocity.x:.3f}")
        self.ang_vel_y_var.set(f"{msg.angular_velocity.y:.3f}")
        self.ang_vel_z_var.set(f"{msg.angular_velocity.z:.3f}")
    
    def quaternion_to_euler(self, x, y, z, w):
        """쿼터니언을 오일러 각(roll, pitch, yaw)으로 변환"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
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
        
    def shutdown(self):
        """노드와 GUI를 안전하게 종료"""
        self.get_logger().info('Shutting down node.')
        self.destroy_node()
        self.gui_root.quit()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    # GUI와 ROS 노드를 별도의 스레드에서 실행
    root = tk.Tk()
    node = ForceTorqueControllerNode(root)
    
    # ROS 노드를 실행할 스레드
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    # 메인 스레드에서는 GUI 실행
    root.mainloop()

if __name__ == '__main__':
    main()