#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import tkinter as tk
from tkinter import ttk
import threading
import math

class PoseControlNode(Node):
    """위성의 위치와 자세를 제어하는 GUI ROS 2 노드"""
    
    def __init__(self, gui_root):
        super().__init__('pose_control_camtest')
        
        # SetEntityPose 서비스 클라이언트 생성
        self.pose_client = self.create_client(
            SetEntityPose,
            '/world/space_world/set_pose'
        )
        
        # Odometry 구독자 생성 (현재 위치 모니터링)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/model/nasa_satellite5/odometry',
            self.odom_callback,
            10
        )
        
        self.gui_root = gui_root
        
        # GUI 변수들 - 위치
        self.pos_x = tk.DoubleVar(value=0.0)
        self.pos_y = tk.DoubleVar(value=0.0)
        self.pos_z = tk.DoubleVar(value=0.0)
        
        # GUI 변수들 - 자세 (오일러 각도)
        self.roll = tk.DoubleVar(value=0.0)
        self.pitch = tk.DoubleVar(value=0.0)
        self.yaw = tk.DoubleVar(value=0.0)
        
        # 현재 상태 표시 변수들
        self.current_x = tk.StringVar(value="0.000")
        self.current_y = tk.StringVar(value="0.000")
        self.current_z = tk.StringVar(value="0.000")
        self.current_roll = tk.StringVar(value="0.0")
        self.current_pitch = tk.StringVar(value="0.0")
        self.current_yaw = tk.StringVar(value="0.0")
        
        # 프리셋 정의 (world 파일의 주석에서 가져온 값들)
        self.presets = {
            "Origin (0m)": (0, 0, 0, 0, 0, 0),
            "33km - Far Detection": (0, 33000, 0, 0, 0, 1.58),
            "22km - Initial Track": (0, 22000, 0, 0, 0, 0),
            "10km - Mid Range": (0, 10000, 0, 0, 0, 0),
            "5km - Approach": (0, 5000, 0, 0, 0, 0),
            "2km - Close Range": (0, 2000, 0, 0, 0.3, 1.78),
            "1km - Detailed View": (0, 1000, 0, 0, 0, 0),
            "300m - Structure Clear": (0, 300, 0, 0, 0, 1.58),
            "150m - Very Clear": (0, 150, 0, 0, 0.3, 1.78),
            "100m - Extreme Detail": (0, 100, 0, 0, 0, 0),
            "90m - Maneuvering": (0, 90, 0, 0, 0, 0),
            "15m - NFOV/WFOV Switch": (0, 15, 0, 0, 0, 0),
            "6m - Final Approach": (0, 6, 0, 0, 0, 0),
            "2m - Docking Ready": (0, 2, 0, 0, 0, 0),
        }
        
        self.setup_gui()
        
        # 서비스가 준비될 때까지 대기
        while not self.pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SetEntityPose service...')
        self.get_logger().info('SetEntityPose service is ready.')

    def setup_gui(self):
        """Tkinter GUI 설정"""
        self.gui_root.title("Satellite Pose Control - Camera Test")
        
        # 메인 컨테이너
        main_container = ttk.Frame(self.gui_root, padding="10")
        main_container.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 왼쪽: 제어 패널
        control_frame = ttk.Frame(main_container)
        control_frame.grid(row=0, column=0, padx=5, sticky="nsew")
        
        # 오른쪽: 현재 상태 패널
        status_frame = ttk.Frame(main_container)
        status_frame.grid(row=0, column=1, padx=5, sticky="nsew")
        
        # --- 제어 패널 ---
        ttk.Label(control_frame, text="POSE CONTROL", font=("Arial", 14, "bold")).grid(row=0, column=0, columnspan=2, pady=10)
        
        # 프리셋 선택
        preset_frame = ttk.LabelFrame(control_frame, text="Presets", padding="10")
        preset_frame.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        
        self.preset_var = tk.StringVar()
        self.preset_combo = ttk.Combobox(preset_frame, textvariable=self.preset_var, 
                                         values=list(self.presets.keys()), 
                                         width=30, state="readonly")
        self.preset_combo.grid(row=0, column=0, padx=5)
        self.preset_combo.bind('<<ComboboxSelected>>', self.on_preset_selected)
        
        ttk.Button(preset_frame, text="Apply Preset", 
                  command=self.apply_preset).grid(row=0, column=1, padx=5)
        
        # 위치 입력
        position_frame = ttk.LabelFrame(control_frame, text="Position (m)", padding="10")
        position_frame.grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        
        ttk.Label(position_frame, text="X:").grid(row=0, column=0, sticky="w", padx=5)
        ttk.Entry(position_frame, textvariable=self.pos_x, width=15).grid(row=0, column=1, padx=5)
        
        ttk.Label(position_frame, text="Y:").grid(row=1, column=0, sticky="w", padx=5)
        ttk.Entry(position_frame, textvariable=self.pos_y, width=15).grid(row=1, column=1, padx=5)
        
        ttk.Label(position_frame, text="Z:").grid(row=2, column=0, sticky="w", padx=5)
        ttk.Entry(position_frame, textvariable=self.pos_z, width=15).grid(row=2, column=1, padx=5)
        
        # 자세 입력 (라디안)
        orientation_frame = ttk.LabelFrame(control_frame, text="Orientation (rad)", padding="10")
        orientation_frame.grid(row=3, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        
        ttk.Label(orientation_frame, text="Roll:").grid(row=0, column=0, sticky="w", padx=5)
        ttk.Entry(orientation_frame, textvariable=self.roll, width=15).grid(row=0, column=1, padx=5)
        
        ttk.Label(orientation_frame, text="Pitch:").grid(row=1, column=0, sticky="w", padx=5)
        ttk.Entry(orientation_frame, textvariable=self.pitch, width=15).grid(row=1, column=1, padx=5)
        
        ttk.Label(orientation_frame, text="Yaw:").grid(row=2, column=0, sticky="w", padx=5)
        ttk.Entry(orientation_frame, textvariable=self.yaw, width=15).grid(row=2, column=1, padx=5)
        
        # 각도 변환 도우미
        convert_frame = ttk.Frame(control_frame)
        convert_frame.grid(row=4, column=0, columnspan=2, pady=5)
        ttk.Label(convert_frame, text="Quick Convert:", font=("Arial", 9)).grid(row=0, column=0, padx=5)
        ttk.Label(convert_frame, text="90° = 1.57 rad | 180° = 3.14 rad", 
                 font=("Arial", 9, "italic")).grid(row=0, column=1)
        
        # 제어 버튼
        button_frame = ttk.Frame(control_frame, padding="10")
        button_frame.grid(row=5, column=0, columnspan=2, sticky="ew")
        
        ttk.Button(button_frame, text="Apply Pose", 
                  command=self.apply_pose, style="Accent.TButton").pack(side="left", padx=5)
        ttk.Button(button_frame, text="Reset to Origin", 
                  command=self.reset_to_origin).pack(side="left", padx=5)
        ttk.Button(button_frame, text="Quit", 
                  command=self.shutdown).pack(side="right", padx=5)
        
        # --- 상태 모니터링 패널 ---
        ttk.Label(status_frame, text="CURRENT STATE", font=("Arial", 14, "bold")).grid(row=0, column=0, pady=10)
        
        # 현재 위치
        current_pos_frame = ttk.LabelFrame(status_frame, text="Current Position (m)", padding="10")
        current_pos_frame.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        self.create_status_display(current_pos_frame, ["X:", "Y:", "Z:"],
                                  [self.current_x, self.current_y, self.current_z])
        
        # 현재 자세
        current_ori_frame = ttk.LabelFrame(status_frame, text="Current Orientation (deg)", padding="10")
        current_ori_frame.grid(row=2, column=0, padx=5, pady=5, sticky="ew")
        self.create_status_display(current_ori_frame, ["Roll:", "Pitch:", "Yaw:"],
                                  [self.current_roll, self.current_pitch, self.current_yaw])
        
        # 정보 패널
        info_frame = ttk.LabelFrame(status_frame, text="Info", padding="10")
        info_frame.grid(row=3, column=0, padx=5, pady=5, sticky="ew")
        
        info_text = """Target: intel_sat_dummy
Cameras:
- NFOV: 5.5° FOV (long range)
- WFOV: 19.5° FOV (mid range)  
- Docking: 68° FOV (close range)

Distance ranges:
• 33km-10km: Detection
• 10km-1km: Tracking
• 1km-15m: Approach
• 15m-2m: Final/Docking"""
        
        info_label = ttk.Label(info_frame, text=info_text, font=("Courier", 9), justify="left")
        info_label.grid(row=0, column=0, padx=5, pady=5)
        
    def create_status_display(self, parent, labels, variables):
        """상태 표시 라벨 생성"""
        for i, (label, var) in enumerate(zip(labels, variables)):
            ttk.Label(parent, text=label, width=8).grid(row=i, column=0, sticky="w", padx=5)
            display = ttk.Label(parent, textvariable=var, width=15,
                              relief="sunken", anchor="e", font=("Courier", 10))
            display.grid(row=i, column=1, sticky="ew", padx=5, pady=2)
    
    def on_preset_selected(self, event):
        """프리셋이 선택되면 값을 입력 필드에 표시"""
        selected = self.preset_var.get()
        if selected in self.presets:
            x, y, z, roll, pitch, yaw = self.presets[selected]
            self.pos_x.set(x)
            self.pos_y.set(y)
            self.pos_z.set(z)
            self.roll.set(roll)
            self.pitch.set(pitch)
            self.yaw.set(yaw)
    
    def apply_preset(self):
        """선택된 프리셋을 적용"""
        selected = self.preset_var.get()
        if selected in self.presets:
            self.on_preset_selected(None)
            self.apply_pose()
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """오일러 각을 쿼터니언으로 변환"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        
        return q
    
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
    
    def apply_pose(self):
        """현재 입력된 위치와 자세를 적용"""
        request = SetEntityPose.Request()
        request.entity.name = 'intel_sat_dummy'
        request.entity.type = 2  # MODEL type
        
        # Pose 설정
        pose = Pose()
        pose.position.x = self.pos_x.get()
        pose.position.y = self.pos_y.get()
        pose.position.z = self.pos_z.get()
        
        # 오일러 각을 쿼터니언으로 변환
        pose.orientation = self.euler_to_quaternion(
            self.roll.get(), 
            self.pitch.get(), 
            self.yaw.get()
        )
        
        request.pose = pose
        
        # 서비스 호출
        future = self.pose_client.call_async(request)
        future.add_done_callback(self.pose_response_callback)
        
        self.get_logger().info(f'Setting pose: pos=({pose.position.x:.1f}, {pose.position.y:.1f}, {pose.position.z:.1f}), ' +
                              f'rot=({self.roll.get():.2f}, {self.pitch.get():.2f}, {self.yaw.get():.2f})')
    
    def pose_response_callback(self, future):
        """SetEntityPose 서비스 응답 콜백"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Pose set successfully')
            else:
                self.get_logger().error('Failed to set pose')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    def reset_to_origin(self):
        """원점으로 리셋"""
        self.pos_x.set(0.0)
        self.pos_y.set(0.0)
        self.pos_z.set(0.0)
        self.roll.set(0.0)
        self.pitch.set(0.0)
        self.yaw.set(0.0)
        self.apply_pose()
    
    def odom_callback(self, msg):
        """Odometry 메시지 콜백 - 현재 위치와 자세 업데이트"""
        # 위치 업데이트
        self.current_x.set(f"{msg.pose.pose.position.x:.3f}")
        self.current_y.set(f"{msg.pose.pose.position.y:.3f}")
        self.current_z.set(f"{msg.pose.pose.position.z:.3f}")
        
        # 자세 업데이트 (쿼터니언을 오일러 각으로 변환)
        q = msg.pose.pose.orientation
        roll, pitch, yaw = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
        
        # 라디안을 도로 변환하여 표시
        self.current_roll.set(f"{math.degrees(roll):.1f}")
        self.current_pitch.set(f"{math.degrees(pitch):.1f}")
        self.current_yaw.set(f"{math.degrees(yaw):.1f}")
    
    def shutdown(self):
        """노드와 GUI를 안전하게 종료"""
        self.get_logger().info('Shutting down pose control node.')
        self.destroy_node()
        self.gui_root.quit()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    # GUI와 ROS 노드 생성
    root = tk.Tk()
    node = PoseControlNode(root)
    
    # ROS 노드를 별도 스레드에서 실행
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    # 메인 스레드에서 GUI 실행
    root.mainloop()

if __name__ == '__main__':
    main()