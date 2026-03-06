#!/usr/bin/env python3
"""
LiDAR Orbit Scanner - 타겟 주변을 원형 궤도로 돌면서 LiDAR 스캔 수행.
odom -> base_link TF를 퍼블리시하여 매핑 노드와 연동.
"""
import math
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class LidarOrbitScanner(Node):
    def __init__(self):
        super().__init__('lidar_orbit_scanner')

        self.declare_parameter('entity_name', 'controla_prototype_1')
        self.declare_parameter('world_name', 'lidar_test_world')
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 3.0)
        self.declare_parameter('target_z', 0.0)
        self.declare_parameter('orbit_radius', 4.0)
        self.declare_parameter('orbit_speed', 0.1)  # rad/s
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('lidar_frame',
                               'controla_prototype_1/controla_prototype_1_link/lidar_3d')

        self.entity_name = self.get_parameter('entity_name').value
        world_name = self.get_parameter('world_name').value
        self.target = [
            self.get_parameter('target_x').value,
            self.get_parameter('target_y').value,
            self.get_parameter('target_z').value,
        ]
        self.radius = self.get_parameter('orbit_radius').value
        self.speed = self.get_parameter('orbit_speed').value
        update_rate = self.get_parameter('update_rate').value
        lidar_frame = self.get_parameter('lidar_frame').value

        # SetEntityPose 서비스
        service_name = f'/world/{world_name}/set_pose'
        self.client = self.create_client(SetEntityPose, service_name)
        self.get_logger().info(f'Waiting for {service_name}...')
        self.client.wait_for_service(timeout_sec=30.0)
        self.get_logger().info('Service ready.')

        # TF broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # base_link -> lidar_frame 정적 TF (SDF의 LiDAR pose)
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'base_link'
        static_tf.child_frame_id = lidar_frame
        static_tf.transform.translation.x = -0.11677
        static_tf.transform.translation.y = 0.5224
        static_tf.transform.translation.z = 0.2269
        # yaw = π/2
        static_tf.transform.rotation.z = 0.7071068
        static_tf.transform.rotation.w = 0.7071068
        self.static_tf_broadcaster.sendTransform(static_tf)

        self.angle = 0.0
        self.dt = 1.0 / update_rate
        self.timer = self.create_timer(self.dt, self.orbit_step)
        self.get_logger().info(
            f'Orbiting around {self.target}, r={self.radius}m, '
            f'speed={self.speed} rad/s (~{2*math.pi/self.speed:.0f}s/rev)')

    def orbit_step(self):
        self.angle += self.speed * self.dt

        # 타겟 중심 원형 궤도
        x = self.target[0] + self.radius * math.cos(self.angle)
        y = self.target[1] + self.radius * math.sin(self.angle)
        z = self.target[2]

        # LiDAR(+Y방향)가 타겟을 향하도록: 위성 yaw = atan2(dy,dx) - π/2
        dx = self.target[0] - x
        dy = self.target[1] - y
        yaw = math.atan2(dy, dx) - math.pi / 2
        qz = math.sin(yaw / 2)
        qw = math.cos(yaw / 2)

        # Gazebo pose 업데이트
        req = SetEntityPose.Request()
        req.entity.name = self.entity_name
        req.pose.position.x = x
        req.pose.position.y = y
        req.pose.position.z = z
        req.pose.orientation.z = qz
        req.pose.orientation.w = qw
        self.client.call_async(req)

        # odom -> base_link TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = LidarOrbitScanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
