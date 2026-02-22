#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class CheckPoints(Node):
    def __init__(self):
        super().__init__('check_points')
        self.sub = self.create_subscription(
            PointCloud2,
            '/lidar/points_raw/points',
            self.callback,
            10
        )
        self.count = 0
        
    def callback(self, msg):
        self.count += 1
        # 첫 4바이트만 확인 (첫 번째 x 좌표)
        first_x = float(int.from_bytes(msg.data[0:4], 'little'))
        print(f"Frame {self.count}: First X = {first_x:.2f}")

def main():
    rclpy.init()
    node = CheckPoints()
    rclpy.spin(node)

if __name__ == '__main__':
    main()