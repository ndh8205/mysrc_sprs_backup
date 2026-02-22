#!/usr/bin/env python3
"""ROS2 기반 토크 인가 + 데이터 로깅 (sim_time 동기화)"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
import math
import csv
from datetime import datetime


class GazeboTestNode(Node):
    def __init__(self):
        super().__init__('gazebo_test_node')
        
        # Publishers - 토크 명령
        self.torque_pub = self.create_publisher(
            Float64, '/model/kari_arm/joint/joint1/cmd_force', 10)
        
        # Subscribers - 데이터 수집
        self.joint_sub = self.create_subscription(
            JointState, '/world/kari_arm_world/model/kari_arm/joint_state',
            self.joint_callback, 10)
        
        self.pose_sub = self.create_subscription(
            TFMessage, '/world/kari_arm_world/pose/info',
            self.pose_callback, 10)
        
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_callback, 10)
        
        # 데이터 저장
        self.data = []
        self.current_pose = None
        self.sim_time = 0.0
        self.start_time = None
        self.duration = 5.0
        self.running = False
        self.last_log_time = -1.0
        self.dt = 0.01  # 로깅 간격
        
        self.get_logger().info('노드 시작. joint_state 수신 대기...')
        
    def clock_callback(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        
    def pose_callback(self, msg):
        if len(msg.transforms) > 0:
            t = msg.transforms[0].transform.translation
            r = msg.transforms[0].transform.rotation
            self.current_pose = {
                'position': [t.x, t.y, t.z],
                'quaternion': [r.w, r.x, r.y, r.z]
            }
    
    def joint_callback(self, msg):
        """joint_state 수신 시 동기화된 로깅"""
        # 메시지 타임스탬프 사용
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # 시작 전 대기 (3초)
        if self.start_time is None:
            if msg_time > 3.0:
                self.start_time = msg_time
                self.running = True
                self.get_logger().info(f'테스트 시작! (sim_time: {msg_time:.2f})')
            return
        
        if not self.running:
            return
        
        # 경과 시간
        t = msg_time - self.start_time
        
        if t >= self.duration:
            self.finish_test()
            return
        
        # dt 간격으로 로깅 (sim_time 기준)
        if t - self.last_log_time >= self.dt - 0.001:  # 약간의 허용 오차
            self.last_log_time = t
            
            # 해당 시점의 토크 계산
            torque = 0.1 * math.sin(t)
            
            # 데이터 로깅
            if self.current_pose:
                self.data.append({
                    'time': t,
                    'torque_cmd': torque,
                    'joint_pos': list(msg.position),
                    'joint_vel': list(msg.velocity) if msg.velocity else [0]*7,
                    'sat_pos': self.current_pose['position'].copy(),
                    'sat_quat': self.current_pose['quaternion'].copy()
                })
            
            # 매 1초 출력
            if int(t * 10) % 10 == 0:
                q1 = msg.position[0] if msg.position else 0
                self.get_logger().info(f't={t:.2f}s | tau={torque:+.4f} | q1={math.degrees(q1):.2f}°')
        
        # 토크 발행 (매 callback마다)
        t_for_torque = msg_time - self.start_time
        if 0 <= t_for_torque < self.duration:
            torque = 0.1 * math.sin(t_for_torque)
            torque_msg = Float64()
            torque_msg.data = torque
            self.torque_pub.publish(torque_msg)
    
    def finish_test(self):
        self.running = False
        
        # 토크 0
        msg = Float64()
        msg.data = 0.0
        self.torque_pub.publish(msg)
        
        # CSV 저장
        filename = f"gazebo_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'torque_cmd', 
                           'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7',
                           'qd1', 'qd2', 'qd3', 'qd4', 'qd5', 'qd6', 'qd7',
                           'sat_x', 'sat_y', 'sat_z',
                           'sat_qw', 'sat_qx', 'sat_qy', 'sat_qz'])
            
            for d in self.data:
                row = [d['time'], d['torque_cmd']]
                row.extend(d['joint_pos'][:7] if len(d['joint_pos']) >= 7 else d['joint_pos'] + [0]*(7-len(d['joint_pos'])))
                row.extend(d['joint_vel'][:7] if len(d['joint_vel']) >= 7 else d['joint_vel'] + [0]*(7-len(d['joint_vel'])))
                row.extend(d['sat_pos'])
                row.extend(d['sat_quat'])
                writer.writerow(row)
        
        self.get_logger().info(f'테스트 완료! {len(self.data)} 샘플')
        self.get_logger().info(f'저장: {filename}')
        
        raise SystemExit


def main():
    rclpy.init()
    node = GazeboTestNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()