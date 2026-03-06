#!/usr/bin/env python3
"""
3D PointCloud Mapper - LiDAR PointCloud2를 odom 프레임으로 변환하여 누적.
누적된 3D 맵을 /map_cloud 토픽으로 퍼블리시.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import tf2_ros
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class PointcloudMapper(Node):
    def __init__(self):
        super().__init__('pointcloud_mapper')

        self.declare_parameter('input_topic',
                               '/controla_prototype_1/lidar/points/points')
        self.declare_parameter('output_topic', '/map_cloud')
        self.declare_parameter('fixed_frame', 'odom')
        self.declare_parameter('voxel_size', 0.05)  # m, 다운샘플링
        self.declare_parameter('max_points', 500000)
        self.declare_parameter('publish_rate', 1.0)  # Hz

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.fixed_frame = self.get_parameter('fixed_frame').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.max_points = self.get_parameter('max_points').value
        pub_rate = self.get_parameter('publish_rate').value

        # TF buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 누적 포인트 저장소
        self.accumulated_points = np.empty((0, 3), dtype=np.float32)

        # 구독/발행
        self.sub = self.create_subscription(
            PointCloud2, input_topic, self.cloud_callback, 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)
        self.pub = self.create_publisher(PointCloud2, output_topic, qos)

        self.timer = self.create_timer(1.0 / pub_rate, self.publish_map)
        self.scan_count = 0
        self.get_logger().info(
            f'3D mapper: {input_topic} -> {output_topic} '
            f'(voxel={self.voxel_size}m, max={self.max_points}pts)')

    def cloud_callback(self, msg: PointCloud2):
        # TF 조회: sensor_frame -> fixed_frame
        try:
            tf = self.tf_buffer.lookup_transform(
                self.fixed_frame, msg.header.frame_id,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=2.0)
            return

        # PointCloud2 -> numpy xyz
        points = []
        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            points.append([p[0], p[1], p[2]])

        if not points:
            return

        pts = np.array(points, dtype=np.float32)

        # inf/nan 제거
        valid = np.isfinite(pts).all(axis=1)
        pts = pts[valid]
        if len(pts) == 0:
            return

        # TF 적용: 회전 + 이동
        t = tf.transform.translation
        r = tf.transform.rotation
        rot_matrix = self._quat_to_matrix(r.x, r.y, r.z, r.w)
        translation = np.array([t.x, t.y, t.z], dtype=np.float32)

        transformed = (rot_matrix @ pts.T).T + translation

        # 누적
        self.accumulated_points = np.vstack([self.accumulated_points, transformed])
        self.scan_count += 1

        # 일정 간격으로 복셀 다운샘플링
        if self.scan_count % 10 == 0:
            self._voxel_downsample()

        # 최대 포인트 수 초과 시 오래된 것부터 제거
        if len(self.accumulated_points) > self.max_points:
            self.accumulated_points = self.accumulated_points[-self.max_points:]

    def publish_map(self):
        if len(self.accumulated_points) == 0:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.fixed_frame

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_msg = pc2.create_cloud(header, fields, self.accumulated_points.tolist())
        self.pub.publish(cloud_msg)
        self.get_logger().info(
            f'Map published: {len(self.accumulated_points)} points '
            f'({self.scan_count} scans)', throttle_duration_sec=5.0)

    def _voxel_downsample(self):
        if len(self.accumulated_points) == 0:
            return
        vs = self.voxel_size
        # 복셀 인덱스 계산 후 중복 제거
        voxel_idx = np.floor(self.accumulated_points / vs).astype(np.int32)
        _, unique_indices = np.unique(voxel_idx, axis=0, return_index=True)
        self.accumulated_points = self.accumulated_points[unique_indices]

    @staticmethod
    def _quat_to_matrix(qx, qy, qz, qw):
        """쿼터니언 -> 3x3 회전 행렬"""
        return np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
        ], dtype=np.float32)


def main(args=None):
    rclpy.init(args=args)
    node = PointcloudMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
