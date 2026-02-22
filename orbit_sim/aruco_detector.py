#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math


class HomographyDecomposition:
    def __init__(self):
        self.name = "HomographyDecomposition"
    
    def order_points(self, pts):
        """4개 점을 좌상단, 우상단, 우하단, 좌하단 순서로 정렬"""
        pts = np.array(pts).reshape(4, 2)
        
        # 점들의 합과 차이 계산
        s = pts.sum(axis=1)
        diff = np.diff(pts, axis=1)
        
        # 순서대로 정렬
        ordered = np.zeros((4, 2), dtype=np.float64)
        ordered[0] = pts[np.argmin(s)]  # 좌상단
        ordered[2] = pts[np.argmax(s)]  # 우하단
        ordered[1] = pts[np.argmin(diff)]  # 우상단
        ordered[3] = pts[np.argmax(diff)]  # 좌하단
        
        return ordered
    
    def robust_homography_decomposition(self, model_pts_2d, ordered_corners, camera_matrix):
        """강건한 호모그래피 분해"""
        # getPerspectiveTransform을 위한 float32 변환
        src_pts = model_pts_2d.astype(np.float32)
        dst_pts = ordered_corners.astype(np.float32)
        
        # 호모그래피 계산
        H = cv2.getPerspectiveTransform(src_pts, dst_pts)
        H = H.astype(np.float64)
        
        K_inv = np.linalg.inv(camera_matrix)
        
        h1 = H[:, 0]
        h2 = H[:, 1]
        h3 = H[:, 2]
        
        norm_h1 = np.linalg.norm(K_inv @ h1)
        if norm_h1 < 1e-6:
            raise ValueError("Homography decomposition unstable: norm of K_inv @ h1 too small")
        
        lambda_val = 1.0 / norm_h1
        r1 = lambda_val * K_inv @ h1
        r2 = lambda_val * K_inv @ h2
        t_init = lambda_val * K_inv @ h3
        r3 = np.cross(r1, r2)
        
        R_init = np.column_stack((r1, r2, r3))
        
        # SVD를 사용한 적절한 회전 행렬 보장
        U, S, Vt = np.linalg.svd(R_init)
        R_init = U @ Vt
        
        # determinant가 1이 되도록 보장
        if np.linalg.det(R_init) < 0:
            R_init[:, 2] = -R_init[:, 2]
        
        return R_init, t_init
    
    def compute_reprojection_error(self, object_pts, image_pts, rvec, tvec, camera_matrix, dist_coeffs):
        """재투영 오차 계산"""
        projected, _ = cv2.projectPoints(object_pts, rvec, tvec, camera_matrix, dist_coeffs)
        projected = projected.reshape(-1, 2)
        
        error = 0.0
        for i in range(len(projected)):
            dx = projected[i, 0] - image_pts[i, 0]
            dy = projected[i, 1] - image_pts[i, 1]
            error += np.sqrt(dx*dx + dy*dy)
        
        return error / len(projected)
    
    def orthogonal_iteration(self, R_init, t_init, object_pts, v_list, n_iters=20):
        """직교 반복법"""
        R_iter = R_init.copy()
        t_iter = t_init.copy()
        
        for iter in range(n_iters):
            # t 업데이트 (R 고정)
            A = np.zeros((3, 3), dtype=np.float64)
            b = np.zeros((3, 1), dtype=np.float64)
            
            for i in range(4):
                v_i = v_list[i].reshape(3, 1)
                F_i = v_i @ v_i.T / (v_i.T @ v_i + 1e-8)
                I_minus_F = np.eye(3) - F_i
                
                p_i = object_pts[i].reshape(3, 1)
                
                A += I_minus_F
                b += -I_minus_F @ R_iter @ p_i
            
            t_iter = np.linalg.solve(A, b)
            
            # R 업데이트 (t 고정)
            Q = []
            P = []
            
            for i in range(4):
                v_i = v_list[i].reshape(3, 1)
                F_i = v_i @ v_i.T / (v_i.T @ v_i + 1e-8)
                
                p_i = object_pts[i].reshape(3, 1)
                X_i = R_iter @ p_i + t_iter
                q_i = F_i @ X_i
                
                Q.append(q_i)
                P.append(p_i)
            
            # 중심점 계산
            Q_mean = np.mean(Q, axis=0)
            P_mean = np.mean(P, axis=0)
            
            # 공분산 행렬 계산
            H_cov = np.zeros((3, 3), dtype=np.float64)
            for i in range(4):
                q_centered = Q[i] - Q_mean
                p_centered = P[i] - P_mean
                H_cov += p_centered @ q_centered.T
            
            # SVD를 통한 최적 회전 찾기
            U, S, Vt = np.linalg.svd(H_cov)
            R_new = Vt.T @ U.T
            
            if np.linalg.det(R_new) < 0:
                Vt_fixed = Vt.copy()
                Vt_fixed[2, :] = -Vt_fixed[2, :]
                R_new = Vt_fixed.T @ U.T
            
            R_iter = R_new.copy()
        
        return R_iter, t_iter.flatten()
    
    def fix_pose_ambiguities(self, R, t, object_pts, ordered_corners, camera_matrix, dist_coeffs):
        """포즈 모호성 해결"""
        # 카메라를 향하는 법선 벡터 확인
        normal1 = np.array([R[0, 2], R[1, 2], R[2, 2]])
        
        # 태그 중심이 카메라 앞에 있는지 확인
        is_in_front = t[2] > 0
        
        # 법선이 카메라를 향하는지 확인
        normal_towards_camera = normal1[2] < 0
        
        # 첫 번째 후보
        R1 = R.copy()
        t1 = t.copy()
        
        # 두 번째 후보 - 180도 회전
        R2 = R.copy()
        R2[:, 2] = -R2[:, 2]  # Z축 반전
        R2[:, 0] = -R2[:, 0]  # X축 반전
        t2 = t.copy()
        
        # 태그가 카메라 뒤에 있는 경우
        if not is_in_front:
            Rz_180 = np.array([[-1, 0, 0],
                              [0, -1, 0],
                              [0, 0, 1]], dtype=np.float64)
            R1 = R1 @ Rz_180
            R2 = R2 @ Rz_180
            t1 = -t1
            t2 = -t2
        
        # 재투영 오차 계산
        rvec1 = cv2.Rodrigues(R1)[0]
        rvec2 = cv2.Rodrigues(R2)[0]
        
        error1 = self.compute_reprojection_error(object_pts, ordered_corners, rvec1, t1, camera_matrix, dist_coeffs)
        error2 = self.compute_reprojection_error(object_pts, ordered_corners, rvec2, t2, camera_matrix, dist_coeffs)
        
        # 더 작은 오차를 가진 포즈 선택
        if error2 < error1:
            selected_R = R2
            selected_t = t2
        else:
            selected_R = R1
            selected_t = t1
        
        # 최종 검증
        final_normal = np.array([selected_R[0, 2], selected_R[1, 2], selected_R[2, 2]])
        
        if final_normal[2] > 0 and selected_t[2] > 0:
            selected_R[:, 2] = -selected_R[:, 2]
            selected_R[:, 0] = -selected_R[:, 0]
        
        return selected_R, selected_t
    
    def rotation_matrix_to_euler_angles(self, R):
        """회전 행렬을 Euler 각도로 변환 (degrees)"""
        sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
        singular = sy < 1e-6
        
        if not singular:
            x = np.arctan2(R[2,1], R[2,2])
            y = np.arctan2(-R[2,0], sy)
            z = np.arctan2(R[1,0], R[0,0])
        else:
            x = np.arctan2(-R[1,2], R[1,1])
            y = np.arctan2(-R[2,0], sy)
            z = 0
        
        return np.degrees([x, y, z])
    
    def estimate_pose(self, image_corners, camera_matrix, dist_coeffs, marker_size):
        """포즈 추정 메인 함수"""
        result = {
            'success': False,
            'method_name': self.name,
            'rotation_matrix': None,
            'tvec': None,
            'rvec': None,
            'reprojection_error': float('inf'),
            'euler_angles': None
        }
        
        if len(image_corners) != 4:
            return result
        
        try:
            # 코너 정렬
            ordered_corners = self.order_points(image_corners)
            
            # 모델 좌표
            s = marker_size / 2.0
            model_pts_2d = np.array([
                [-s, s],   # 좌상단
                [s, s],    # 우상단
                [s, -s],   # 우하단
                [-s, -s]   # 좌하단
            ], dtype=np.float64)
            
            # 3D 객체 포인트
            object_pts = np.zeros((4, 3), dtype=np.float64)
            object_pts[:, :2] = model_pts_2d
            
            # 호모그래피 분해
            R_init, t_init = self.robust_homography_decomposition(model_pts_2d, ordered_corners, camera_matrix)
            
            # 정규화된 레이 벡터 계산
            fx = camera_matrix[0, 0]
            fy = camera_matrix[1, 1]
            cx = camera_matrix[0, 2]
            cy = camera_matrix[1, 2]
            
            v_list = []
            for i in range(4):
                u = ordered_corners[i, 0]
                v = ordered_corners[i, 1]
                ray = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
                v_list.append(ray)
            
            # 직교 반복
            R_opt, t_opt = self.orthogonal_iteration(R_init, t_init, object_pts, v_list)
            
            # 모호성 해결
            R_final, t_final = self.fix_pose_ambiguities(R_opt, t_opt, object_pts, ordered_corners, 
                                                        camera_matrix, dist_coeffs)
            
            # 결과 저장
            result['rotation_matrix'] = R_final
            result['tvec'] = t_final
            result['rvec'] = cv2.Rodrigues(R_final)[0]
            result['euler_angles'] = self.rotation_matrix_to_euler_angles(R_final)
            result['reprojection_error'] = self.compute_reprojection_error(
                object_pts, ordered_corners, result['rvec'], t_final, camera_matrix, dist_coeffs)
            result['success'] = True
            
        except Exception as e:
            # Fallback to OpenCV solvePnP
            s = marker_size / 2.0
            object_pts_3d = np.array([
                [-s, s, 0],
                [s, s, 0],
                [s, -s, 0],
                [-s, -s, 0]
            ], dtype=np.float32)
            
            success, rvec, tvec = cv2.solvePnP(object_pts_3d, image_corners, 
                                              camera_matrix, dist_coeffs, 
                                              flags=cv2.SOLVEPNP_ITERATIVE)
            
            if success:
                result['rvec'] = rvec
                result['tvec'] = tvec.flatten()
                result['rotation_matrix'] = cv2.Rodrigues(rvec)[0]
                result['euler_angles'] = self.rotation_matrix_to_euler_angles(result['rotation_matrix'])
                result['success'] = True
                result['method_name'] = self.name + "_fallback_opencv"
        
        return result


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.get_logger().info("ArUco Marker Detector Started")
        self.bridge = CvBridge()
        
        # Parameter declarations
        self.declare_parameter('marker_id', 0)          # Marker ID
        self.declare_parameter('marker_size', 0.8)      # Default marker size (1.0m)
        self.declare_parameter('camera_frame', 'camera_link')  # Camera frame name
        
        # Marker settings
        self.marker_id = self.get_parameter('marker_id').value
        self.marker_size = self.get_parameter('marker_size').value
        
        # Camera intrinsic parameters - Corrected to match Gazebo FOV
        self.camera_matrix = np.array([
            [554.26, 0, 320.0],  # fx, cx
            [0, 554.26, 240.0],  # fy, cy
            [0, 0, 1]
        ], dtype=np.float32)
        
        # Gazebo doesn't have distortion
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        
        # Dictionary types setup - 모든 딕셔너리 타입
        self.dictionary_types = [
            ('DICT_4X4_50', cv2.aruco.DICT_4X4_50),
            ('DICT_4X4_100', cv2.aruco.DICT_4X4_100),
            ('DICT_4X4_250', cv2.aruco.DICT_4X4_250),
            ('DICT_4X4_1000', cv2.aruco.DICT_4X4_1000),
            ('DICT_5X5_50', cv2.aruco.DICT_5X5_50),
            ('DICT_5X5_100', cv2.aruco.DICT_5X5_100),
            ('DICT_5X5_250', cv2.aruco.DICT_5X5_250),
            ('DICT_5X5_1000', cv2.aruco.DICT_5X5_1000),
            ('DICT_6X6_50', cv2.aruco.DICT_6X6_50),
            ('DICT_6X6_100', cv2.aruco.DICT_6X6_100),
            ('DICT_6X6_250', cv2.aruco.DICT_6X6_250),
            ('DICT_6X6_1000', cv2.aruco.DICT_6X6_1000),
            ('DICT_7X7_50', cv2.aruco.DICT_7X7_50),
            ('DICT_7X7_100', cv2.aruco.DICT_7X7_100),
            ('DICT_7X7_250', cv2.aruco.DICT_7X7_250),
            ('DICT_7X7_1000', cv2.aruco.DICT_7X7_1000),
            ('DICT_ARUCO_ORIGINAL', cv2.aruco.DICT_ARUCO_ORIGINAL)
        ]
        
        # ArUco 파라미터 생성
        self.parameters = cv2.aruco.DetectorParameters_create()
        
        # 모든 딕셔너리 생성
        self.aruco_dicts = {}
        for name, dict_id in self.dictionary_types:
            self.aruco_dicts[name] = cv2.aruco.Dictionary_get(dict_id)
        
        # HomographyDecomposition 포즈 추정기 추가
        self.homography_estimator = HomographyDecomposition()
        
        # Camera image subscription
        self.create_subscription(Image, 'nasa_satellite2/camera', self.image_callback, 10)
        
        # Results data
        self.detected_pose = {
            'roll': 0.0,    # Roll (deg)
            'pitch': 0.0,   # Pitch (deg)
            'yaw': 0.0,     # Yaw (deg)
            'x': 0.0,       # X offset
            'y': 0.0,       # Y offset
            'z': 0.0        # Z offset
        }
        self.detection_success = False
        self.current_dict_name = None
        
    def image_callback(self, msg):
        """Camera image callback function"""
        try:
            # Convert ROS image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            output_frame = frame.copy()
            
            # Convert to grayscale for marker detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # 모든 딕셔너리로 마커 검출 시도
            success = False
            for dict_name, aruco_dict in self.aruco_dicts.items():
                corners, ids, rejected = cv2.aruco.detectMarkers(
                    gray, aruco_dict, parameters=self.parameters)
                
                # 마커가 검출되었는지 확인
                if ids is not None and len(ids) > 0:
                    # 우리의 마커 ID가 검출되었는지 확인
                    marker_idx = np.where(ids == self.marker_id)[0]
                    if len(marker_idx) > 0:
                        # 마커 발견!
                        self.current_dict_name = dict_name
                        
                        # 마커 그리기
                        cv2.aruco.drawDetectedMarkers(output_frame, corners, ids)
                        
                        # 첫 번째 매칭 마커 가져오기
                        idx = marker_idx[0]
                        
                        # 마커 코너에서 포즈 추정
                        marker_corners = corners[idx][0]  # 4x2 배열로 가져오기
                        
                        # HomographyDecomposition으로 포즈 추정
                        pose_result = self.homography_estimator.estimate_pose(
                            marker_corners, self.camera_matrix, self.dist_coeffs, self.marker_size)
                        
                        if pose_result['success']:
                            # 좌표축 그리기
                            cv2.drawFrameAxes(output_frame, self.camera_matrix, 
                                             self.dist_coeffs, pose_result['rvec'], pose_result['tvec'], 
                                             length=0.1, thickness=2)
                            
                            # 검출된 포즈 업데이트
                            euler_angles = pose_result['euler_angles']
                            self.detected_pose['roll'] = euler_angles[0]
                            self.detected_pose['pitch'] = euler_angles[1]
                            self.detected_pose['yaw'] = euler_angles[2]
                            self.detected_pose['x'] = pose_result['tvec'][0]
                            self.detected_pose['y'] = pose_result['tvec'][1]
                            self.detected_pose['z'] = pose_result['tvec'][2]
                            
                            # 텍스트 정보 추가
                            info_text = [
                                f"DETECTED - ID: {self.marker_id} [{self.current_dict_name}]",
                                f"Method: {pose_result['method_name']}",
                                f"Position (m): X:{self.detected_pose['x']:.2f} Y:{self.detected_pose['y']:.2f} Z:{self.detected_pose['z']:.2f}",
                                f"Rotation (deg): R:{self.detected_pose['roll']:.1f} P:{self.detected_pose['pitch']:.1f} Y:{self.detected_pose['yaw']:.1f}",
                            ]
                            
                            success = True
                        else:
                            # OpenCV 기본 방법으로 fallback
                            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                                corners[idx], self.marker_size, 
                                self.camera_matrix, self.dist_coeffs)
                            
                            # 좌표축 그리기
                            cv2.drawFrameAxes(output_frame, self.camera_matrix, 
                                             self.dist_coeffs, rvec[0], tvec[0], 
                                             length=0.1, thickness=2)
                            
                            # 회전 벡터를 회전 행렬로 변환
                            R, _ = cv2.Rodrigues(rvec[0])
                            roll, pitch, yaw = self.rotation_matrix_to_euler_angles(R)
                            
                            # 검출된 포즈 업데이트
                            self.detected_pose['roll'] = math.degrees(roll)
                            self.detected_pose['pitch'] = math.degrees(pitch)
                            self.detected_pose['yaw'] = math.degrees(yaw)
                            self.detected_pose['x'] = tvec[0][0][0]
                            self.detected_pose['y'] = tvec[0][0][1]
                            self.detected_pose['z'] = tvec[0][0][2]
                            
                            # 재투영 오차 계산
                            reproj_error = self.calculate_reprojection_error(
                                marker_corners, rvec[0], tvec[0])
                            
                            # 텍스트 정보 추가
                            info_text = [
                                f"DETECTED - ID: {self.marker_id} [{self.current_dict_name}]",
                                f"Method: OpenCV Default",
                                f"Position (m): X:{self.detected_pose['x']:.2f} Y:{self.detected_pose['y']:.2f} Z:{self.detected_pose['z']:.2f}",
                                f"Rotation (deg): R:{self.detected_pose['roll']:.1f} P:{self.detected_pose['pitch']:.1f} Y:{self.detected_pose['yaw']:.1f}",
                            ]
                            
                            success = True
                        break
            
            if not success:
                info_text = ["No markers detected"]
            
            # 정보 텍스트 표시
            for i, text in enumerate(info_text):
                cv2.putText(output_frame, text, (10, 30 + i * 25), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # 결과 창 표시
            cv2.imshow("Fiducial Marker Detection", output_frame)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def calculate_reprojection_error(self, detected_corners, rvec, tvec):
        """Calculate reprojection error between detected corners and projected points"""
        # Define the marker coordinate system
        half = self.marker_size / 2
        obj_pts = np.array([
            [-half, half, 0],    # Top-left
            [half, half, 0],     # Top-right
            [half, -half, 0],    # Bottom-right
            [-half, -half, 0]    # Bottom-left
        ], dtype=np.float32)
        
        # Project points using the estimated pose
        projected_pts, _ = cv2.projectPoints(
            obj_pts, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        projected_pts = projected_pts.reshape(-1, 2)
        
        # Calculate Euclidean distance between detected and projected points
        error = np.linalg.norm(detected_corners - projected_pts, axis=1)
        return np.mean(error)
            
    def rotation_matrix_to_euler_angles(self, R):
        """Convert 3x3 rotation matrix to Euler angles (roll-pitch-yaw)"""
        # Check for gimbal lock
        if abs(R[2, 0]) >= 1.0 - 1e-6:
            # Gimbal lock case
            if R[2, 0] < 0:  # R31 = -1
                roll = 0
                pitch = math.pi / 2
                yaw = math.atan2(R[0, 1], R[0, 2])
            else:  # R31 = 1
                roll = 0
                pitch = -math.pi / 2
                yaw = math.atan2(-R[0, 1], -R[0, 2])
        else:
            # Normal case
            pitch = -math.asin(R[2, 0])
            roll = math.atan2(R[2, 1] / math.cos(pitch), R[2, 2] / math.cos(pitch))
            yaw = math.atan2(R[1, 0] / math.cos(pitch), R[0, 0] / math.cos(pitch))
            
        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    
    # Cleanup
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()