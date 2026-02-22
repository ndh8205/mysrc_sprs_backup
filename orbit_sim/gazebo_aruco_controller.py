#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ros_gz_interfaces.srv import SetEntityPose
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import numpy as np
import cv2
import math
import time

def eulerAnglesToRotationMatrix(angles):
    """
    Euler 각도 (roll, pitch, yaw, degrees)를 받아 회전 행렬을 계산합니다.
    회전 순서는 R = Rz * Ry * Rx 입니다.
    """
    roll, pitch, yaw = np.deg2rad(angles)
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll),  np.cos(roll)]])
    R_y = np.array([[ np.cos(pitch), 0, np.sin(pitch)],
                    [ 0,             1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw),  np.cos(yaw), 0],
                    [0,           0,            1]])
    return R_z @ R_y @ R_x

def rotationMatrixToQuaternion(R):
    """
    3x3 회전 행렬을 쿼터니언 [qx, qy, qz, qw]로 변환합니다.
    """
    qw = math.sqrt(1 + R[0,0] + R[1,1] + R[2,2]) / 2.0
    qx = (R[2,1] - R[1,2]) / (4.0*qw)
    qy = (R[0,2] - R[2,0]) / (4.0*qw)
    qz = (R[1,0] - R[0,1]) / (4.0*qw)
    return [qx, qy, qz, qw]

class GazeboArucoController(Node):
    def __init__(self):
        super().__init__('gazebo_aruco_controller')
        self.bridge = CvBridge()
        self.get_logger().info("Gazebo Aruco Controller 시작")
        
        # 파라미터 선언
        self.declare_parameter('marker_size', 0.18)
        
        # Satellite2 제어 파라미터 (런치 인자에서 전달)
        self.declare_parameter('satellite2.roll', 169.99)
        self.declare_parameter('satellite2.pitch', 48.04)
        self.declare_parameter('satellite2.yaw', 1.78)
        self.declare_parameter('satellite2.tx', 0.10)
        self.declare_parameter('satellite2.ty', 0.21)
        self.declare_parameter('satellite2.tz', 3.77)
        
        # 잠시 대기하여 서비스 등록을 기다림
        time.sleep(5)
        
        # Ignition Gazebo의 set_pose 서비스 클라이언트
        self.pose_client = self.create_client(SetEntityPose, '/world/space_world/set_pose')
        while not self.pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /world/space_world/set_pose service...')
        
        # 카메라 영상 구독 (예: nasa_satellite/camera)
        self.create_subscription(Image, 'nasa_satellite2/camera', self.image_callback, 10)
        
        # 미리 정의된 marker pose들을 계산
        self.markers = self.compute_marker_poses()
        # 위성 모델 상태 업데이트 (nasa_satellite2 업데이트)
        self.update_model_states()

    def compute_marker_poses(self):
        markers = []
        # Marker ID: 278 (고정값)
        markers.append({
            'id': 278,
            'angles': (162.78, 48.61, -4.06),
            'tvec': np.array([0.27, 0.23, 3.65], dtype=np.float32)
        })
        # Marker ID: 51 (Satellite2 제어용 -> 파라미터 사용)
        roll = self.get_parameter('satellite2.roll').value
        pitch = self.get_parameter('satellite2.pitch').value
        yaw = self.get_parameter('satellite2.yaw').value
        tx = self.get_parameter('satellite2.tx').value
        ty = self.get_parameter('satellite2.ty').value
        tz = self.get_parameter('satellite2.tz').value
        markers.append({
            'id': 51,
            'angles': (roll, pitch, yaw),
            'tvec': np.array([tx, ty, tz], dtype=np.float32)
        })
        for marker in markers:
            R_marker = eulerAnglesToRotationMatrix(marker['angles'])
            marker['R'] = R_marker
            marker['quaternion'] = rotationMatrixToQuaternion(R_marker)
            self.get_logger().info(f"Marker {marker['id']} computed: tvec={marker['tvec']}, quat={marker['quaternion']}")
        return markers

    def update_model_states(self):
        for marker in self.markers:
            if marker['id'] != 51:
                continue
            request = SetEntityPose.Request()
            request.entity.name = "nasa_satellite2"
            request.entity.type = request.entity.MODEL
            new_pose = Pose()
            new_pose.position.x = float(marker['tvec'][0])
            new_pose.position.y = float(marker['tvec'][1])
            new_pose.position.z = float(marker['tvec'][2])
            q = marker['quaternion']
            new_pose.orientation.x = q[0]
            new_pose.orientation.y = q[1]
            new_pose.orientation.z = q[2]
            new_pose.orientation.w = q[3]
            request.pose = new_pose

            future = self.pose_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result is not None and result.success:
                self.get_logger().info("Updated pose for nasa_satellite2 using satellite2 parameters")
            else:
                self.get_logger().error("Failed to update pose for nasa_satellite2")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        marker_size = self.get_parameter('marker_size').value
        half = marker_size / 2
        obj_pts = np.float32([
            [-half, -half, 0.0],
            [ half, -half, 0.0],
            [ half,  half, 0.0],
            [-half,  half, 0.0]
        ])
        camera_matrix = np.array([[876.87763605, 0, 386.14382287],
                                  [0, 845.63404158, 293.58980309],
                                  [0, 0, 1]], dtype=np.float32)
        dist_coeffs = np.array([[-7.23572093e-03],
                                [ 1.16278947e+00],
                                [ 4.31554218e-03],
                                [-3.53908448e-03],
                                [-3.98585306e+00]], dtype=np.float32)
        
        for marker in self.markers:
            rvec, _ = cv2.Rodrigues(marker['R'])
            tvec = marker['tvec'].reshape(3, 1)
            img_pts, _ = cv2.projectPoints(obj_pts, rvec, tvec, camera_matrix, dist_coeffs)
            pts = np.int32(img_pts.reshape(-1, 2))
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
            cv2.putText(frame, f"ID:{marker['id']}", tuple(pts[0]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

        cv2.imshow("Gazebo AR Overlay - ArucoMarkers", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = GazeboArucoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
