#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import time

class LidarBridge(Node):
    def __init__(self):
        super().__init__('lidar_bridge_node')
        
        # 토픽 구독자 (다른 브릿지에서 생성된 토픽)
        self.subscription = self.create_subscription(
            PointCloud2,
            '/world/space_world/model/nasa_satellite3/link/nasa_satellite_link/sensor/lidar_3d/points',
            self.lidar_callback,
            10
        )
        
        # 새로운 토픽으로 발행
        self.publisher = self.create_publisher(
            PointCloud2,
            '/lidar/points',
            10
        )
        
        self.get_logger().info('LiDAR bridge node started')
        
    def lidar_callback(self, msg):
        # 헤더 수정
        new_msg = PointCloud2()
        new_msg.header = Header()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = 'nasa_satellite3_link'
        
        # 나머지 데이터 복사
        new_msg.height = msg.height
        new_msg.width = msg.width
        new_msg.fields = msg.fields
        new_msg.is_bigendian = msg.is_bigendian
        new_msg.point_step = msg.point_step
        new_msg.row_step = msg.row_step
        new_msg.data = msg.data
        new_msg.is_dense = msg.is_dense
        
        self.publisher.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()