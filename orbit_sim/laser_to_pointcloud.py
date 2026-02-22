#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection

class LaserToPointCloud(Node):
    def __init__(self):
        super().__init__('laser_to_pointcloud')
        
        self.laser_projector = LaserProjection()
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar/points',
            self.scan_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            PointCloud2,
            '/nasa_satellite3/pointcloud',
            10
        )
        
        self.get_logger().info('LaserScan to PointCloud2 converter started')
        
    def scan_callback(self, msg):
        # LaserScan을 PointCloud2로 변환
        cloud = self.laser_projector.projectLaser(msg)
        self.publisher.publish(cloud)

def main(args=None):
    rclpy.init(args=args)
    node = LaserToPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()