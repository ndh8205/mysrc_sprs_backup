#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import math
import tkinter as tk
from tkinter import ttk
import time

class ArucoSimulator(Node):
    def __init__(self):
        super().__init__('aruco_simulator')
        self.get_logger().info("ArUco Marker Simulator Started")
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
        
        # ArUco dictionary setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Service client creation
        self.pose_client = self.create_client(SetEntityPose, '/world/space_world/set_pose')
        while not self.pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service: /world/space_world/set_pose')
        
        # Camera image subscription
        self.create_subscription(Image, 'nasa_satellite2/camera', self.image_callback, 10)
        
        # Simulation control variables - visual marker position
        self.marker_pose = {
            'roll': 0.0,    # Roll (deg)
            'pitch': 0.0,   # Pitch (deg)
            'yaw': 0.0,     # Yaw (deg)
            'x': 0.0,       # X offset (OpenCV coordinate: right)
            'y': 0.0,       # Y offset (OpenCV coordinate: down)
            'z': 5.0        # Z offset (OpenCV coordinate: depth, away from camera)
        }
        
        # Marker model offset
        self.marker_model_offset = {
            'x': 0.0,  # Model X offset
            'y': 0.0,  # Model Y offset
            'z': 0.0   # Model Z offset
        }
        
        # Model information
        self.model_world_pos = np.array([-12.0, -10.7, 1.3])  # Base position of nasa_satellite2
        self.model_world_angles = (0, 0, 220.6)  # Base orientation in degrees (3.85 rad = 220.6 deg)
        self.camera_model_pos = np.array([0.6153, 0.4367, 0.1405])  # Camera position relative to model
        self.camera_model_angles = (0, 0, 90)  # Camera orientation relative to model
        
        # Variables for smooth movement
        self.last_update_time = time.time()
        self.update_interval = 0.001  
        self.pending_update = False
        self.service_busy = False
        
        # Calculate camera position in world coordinates
        R_world_model = self.euler_to_rotation_matrix(self.model_world_angles)
        R_model_camera = self.euler_to_rotation_matrix(self.camera_model_angles)
        H_world_model = self.build_homogeneous_matrix(R_world_model, self.model_world_pos)
        H_model_camera = self.build_homogeneous_matrix(R_model_camera, self.camera_model_pos)
        H_world_camera = H_world_model @ H_model_camera
        _, camera_pos = self.decompose_homogeneous_matrix(H_world_camera)
        self.get_logger().info(f"Camera world position: {camera_pos.flatten()}")
        
        # Initial marker model position update
        self.update_marker_model_pose()
        
        # Timer for smooth updates
        self.create_timer(self.update_interval, self.update_timer_callback)
        
        # UI setup
        self.setup_ui()
        
    def setup_ui(self):
        self.root = tk.Tk()
        self.root.title("ArUco Marker Simulator Control")
        #self.root.geometry("500x450")
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill="both", expand=True)
        
        # Title
        ttk.Label(main_frame, text="ArUco Marker Simulator", font=("Arial", 14, "bold")).grid(row=0, column=0, columnspan=4, pady=10)
        
        # Marker position and pose adjustment (OpenCV coordinate system)
        param_frame = ttk.LabelFrame(main_frame, text="Marker Position and Pose (OpenCV Coordinates)")
        param_frame.grid(row=1, column=0, columnspan=4, padx=10, pady=10, sticky="ew")
        
        # Sliders, entry fields, and value display
        self.sliders = {}
        self.entries = {}
        self.value_labels = {}
        self.entry_vars = {}
        row = 0
        
        # X, Y, Z position (meters)
        for param, min_val, max_val, default in [
            ('x', -2.0, 2.0, self.marker_pose['x']),
            ('y', -2.0, 2.0, self.marker_pose['y']),
            ('z', 3.0, 15.0, self.marker_pose['z']),
        ]:
            ttk.Label(param_frame, text=f"{param.upper()} (m):").grid(row=row, column=0, sticky="e", padx=5, pady=2)
            
            # Slider
            self.sliders[param] = ttk.Scale(param_frame, from_=min_val, to=max_val, value=default, 
                                          length=200, orient="horizontal")
            self.sliders[param].grid(row=row, column=1, padx=5, pady=2, sticky="ew")
            
            # Entry field for direct value input
            self.entry_vars[param] = tk.StringVar(value=f"{default:.3f}")
            self.entries[param] = ttk.Entry(param_frame, textvariable=self.entry_vars[param], width=8)
            self.entries[param].grid(row=row, column=2, padx=5, pady=2)
            
            # Value display label
            value_label = ttk.Label(param_frame, text=f"{default:.3f}")
            value_label.grid(row=row, column=3, padx=5, pady=2)
            self.value_labels[param] = value_label
            
            row += 1
        
        # Roll, Pitch, Yaw angles (degrees)
        for param, min_val, max_val, default in [
            ('roll', -180.0, 180.0, self.marker_pose['roll']),
            ('pitch', -180.0, 180.0, self.marker_pose['pitch']),
            ('yaw', -180.0, 180.0, self.marker_pose['yaw']),
        ]:
            ttk.Label(param_frame, text=f"{param.title()} (deg):").grid(row=row, column=0, sticky="e", padx=5, pady=2)
            
            # Slider
            self.sliders[param] = ttk.Scale(param_frame, from_=min_val, to=max_val, value=default, 
                                          length=200, orient="horizontal")
            self.sliders[param].grid(row=row, column=1, padx=5, pady=2, sticky="ew")
            
            # Entry field for direct value input
            self.entry_vars[param] = tk.StringVar(value=f"{default:.1f}")
            self.entries[param] = ttk.Entry(param_frame, textvariable=self.entry_vars[param], width=8)
            self.entries[param].grid(row=row, column=2, padx=5, pady=2)
            
            # Value display label
            value_label = ttk.Label(param_frame, text=f"{default:.1f}")
            value_label.grid(row=row, column=3, padx=5, pady=2)
            self.value_labels[param] = value_label
            
            row += 1
            
        # Set up all bindings after creating widgets
        for param in self.marker_pose.keys():
            # Slider callback
            def make_slider_callback(p):
                def callback(event):
                    value = self.sliders[p].get()
                    # Use appropriate precision for different parameters
                    if p in ['roll', 'pitch', 'yaw']:
                        self.entry_vars[p].set(f"{value:.1f}")
                        self.value_labels[p].config(text=f"{value:.1f}")
                    else:
                        self.entry_vars[p].set(f"{value:.3f}")
                        self.value_labels[p].config(text=f"{value:.3f}")
                    self.marker_pose[p] = value
                    self.request_update()
                return callback
            
            # Entry callback
            def make_entry_callback(p):
                def callback(event):
                    try:
                        value = float(self.entry_vars[p].get())
                        min_val = float(self.sliders[p].cget("from"))
                        max_val = float(self.sliders[p].cget("to"))
                        
                        # Ensure value is within allowed range
                        if value < min_val:
                            value = min_val
                            if p in ['roll', 'pitch', 'yaw']:
                                self.entry_vars[p].set(f"{value:.1f}")
                            else:
                                self.entry_vars[p].set(f"{value:.3f}")
                        elif value > max_val:
                            value = max_val
                            if p in ['roll', 'pitch', 'yaw']:
                                self.entry_vars[p].set(f"{value:.1f}")
                            else:
                                self.entry_vars[p].set(f"{value:.3f}")
                        
                        # Set the actual marker pose value BEFORE updating the slider
                        self.marker_pose[p] = value
                        
                        # Update slider position (must be after setting self.marker_pose)
                        self.sliders[p].set(value)
                        
                        # Update label with appropriate precision
                        if p in ['roll', 'pitch', 'yaw']:
                            self.value_labels[p].config(text=f"{value:.1f}")
                        else:
                            self.value_labels[p].config(text=f"{value:.3f}")
                            
                        # Request an update
                        self.request_update()
                    except ValueError:
                        # Restore previous value if input is invalid
                        if p in ['roll', 'pitch', 'yaw']:
                            self.entry_vars[p].set(f"{self.marker_pose[p]:.1f}")
                        else:
                            self.entry_vars[p].set(f"{self.marker_pose[p]:.3f}")
                return callback
            
            slider_callback = make_slider_callback(param)
            entry_callback = make_entry_callback(param)
            
            # Bind events
            self.sliders[param].bind("<B1-Motion>", slider_callback)
            self.sliders[param].bind("<ButtonRelease-1>", slider_callback)
            self.entries[param].bind("<Return>", entry_callback)
            self.entries[param].bind("<FocusOut>", entry_callback)
        
        # Marker size adjustment
        ttk.Label(param_frame, text=f"Marker Size (m):").grid(row=row, column=0, sticky="e", padx=5, pady=2)
        
        # Size slider
        size_slider = ttk.Scale(param_frame, from_=0.01, to=2.0, value=self.marker_size, 
                              length=200, orient="horizontal")
        size_slider.grid(row=row, column=1, padx=5, pady=2, sticky="ew")
        
        # Size entry field
        size_var = tk.StringVar(value=f"{self.marker_size:.3f}")
        size_entry = ttk.Entry(param_frame, textvariable=size_var, width=8)
        size_entry.grid(row=row, column=2, padx=5, pady=2)
        
        # Value display label
        size_label = ttk.Label(param_frame, text=f"{self.marker_size:.3f}")
        size_label.grid(row=row, column=3, padx=5, pady=2)
        
        # Update functions for size
        def update_size_from_slider(event):
            value = size_slider.get()
            size_var.set(f"{value:.3f}")
            size_label.config(text=f"{value:.3f}")
            self.marker_size = value
            self.request_update()
        
        def update_size_from_entry(event):
            try:
                value = float(size_var.get())
                min_val = float(size_slider.cget("from"))
                max_val = float(size_slider.cget("to"))
                
                # Ensure value is within allowed range
                if value < min_val:
                    value = min_val
                    size_var.set(f"{value:.3f}")
                elif value > max_val:
                    value = max_val
                    size_var.set(f"{value:.3f}")
                
                # Update actual size first, then slider
                self.marker_size = value
                size_slider.set(value)
                size_label.config(text=f"{value:.3f}")
                self.request_update()
            except ValueError:
                # Restore previous value if input is invalid
                size_var.set(f"{self.marker_size:.3f}")
        
        size_slider.bind("<B1-Motion>", update_size_from_slider)
        size_slider.bind("<ButtonRelease-1>", update_size_from_slider)
        size_entry.bind("<Return>", update_size_from_entry)
        size_entry.bind("<FocusOut>", update_size_from_entry)
        
        row += 1
        
        # Quick position buttons
        quick_pos_frame = ttk.LabelFrame(main_frame, text="Quick Position")
        quick_pos_frame.grid(row=2, column=0, columnspan=4, padx=10, pady=5, sticky="ew")
        
        def center_marker():
            # Center the marker in the image
            self.marker_pose['x'] = 0.0
            self.marker_pose['y'] = 0.0
            for param in ['x', 'y']:
                self.sliders[param].set(0.0)
                self.entry_vars[param].set("0.000")
                self.value_labels[param].config(text="0.000")
            self.request_update()
        
        def apply_suggested_adjustments():
            # Get the current image and calculate adjustments
            try:
                frame = self.bridge.imgmsg_to_cv2(
                    self.last_image_msg, desired_encoding="bgr8") if hasattr(self, 'last_image_msg') else None
                
                if frame is None:
                    self.get_logger().warn("No image available for adjustment calculation")
                    return
                
                img_h, img_w = frame.shape[:2]
                img_center = (img_w // 2, img_h // 2)
                
                # Calculate marker corners and center
                half = self.marker_size / 2
                obj_pts = np.float32([
                    [-half, -half, 0],  # Top-left
                    [half, -half, 0],   # Top-right
                    [half, half, 0],    # Bottom-right
                    [-half, half, 0]    # Bottom-left
                ])
                
                tvec = np.array([
                    self.marker_pose['x'],
                    self.marker_pose['y'],
                    self.marker_pose['z']
                ]).reshape(3, 1)
                
                angles = (
                    self.marker_pose['roll'],
                    self.marker_pose['pitch'],
                    self.marker_pose['yaw']
                )
                R = self.euler_to_rotation_matrix(angles)
                rvec, _ = cv2.Rodrigues(R)
                
                img_pts, _ = cv2.projectPoints(obj_pts, rvec, tvec, self.camera_matrix, self.dist_coeffs)
                pts = np.int32(img_pts.reshape(-1, 2))
                marker_center = np.mean(pts, axis=0).astype(int)
                
                # Calculate adjustments
                dx = marker_center[0] - img_center[0]
                dy = marker_center[1] - img_center[1]
                
                # Convert pixel offset to approximate 3D position adjustment
                z = tvec[2, 0]  # Z distance from camera to marker
                fx = self.camera_matrix[0, 0]
                fy = self.camera_matrix[1, 1]
                x_adjust = -dx * z / fx
                y_adjust = -dy * z / fy
                
                # Apply adjustments
                self.marker_pose['x'] += x_adjust
                self.marker_pose['y'] += y_adjust
                
                # Update UI
                for param in ['x', 'y']:
                    value = self.marker_pose[param]
                    self.sliders[param].set(value)
                    self.entry_vars[param].set(f"{value:.3f}")
                    self.value_labels[param].config(text=f"{value:.3f}")
                
                self.request_update()
                self.get_logger().info(f"Applied adjustments: X:{x_adjust:.3f}, Y:{y_adjust:.3f}")
            except Exception as e:
                self.get_logger().error(f"Error applying adjustments: {str(e)}")
                import traceback
                self.get_logger().error(traceback.format_exc())
        
        def set_optimal_distance():
            # Set an optimal distance based on marker size
            optimal_z = 3.0  # Reasonable default
            self.marker_pose['z'] = optimal_z
            self.sliders['z'].set(optimal_z)
            self.entry_vars['z'].set(f"{optimal_z:.3f}")
            self.value_labels['z'].config(text=f"{optimal_z:.3f}")
            self.request_update()
        
        # Add buttons for quick positioning
        center_btn = ttk.Button(quick_pos_frame, text="Center Marker (X=0, Y=0)", command=center_marker)
        center_btn.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        
        adjust_btn = ttk.Button(quick_pos_frame, text="Apply Suggested Adjustments", command=apply_suggested_adjustments)
        adjust_btn.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        
        distance_btn = ttk.Button(quick_pos_frame, text="Set Optimal Distance", command=set_optimal_distance)
        distance_btn.grid(row=0, column=2, padx=5, pady=5, sticky="ew")
        
        # Information display
        info_frame = ttk.LabelFrame(main_frame, text="Information")
        info_frame.grid(row=3, column=0, columnspan=4, padx=10, pady=10, sticky="ew")
        
        info_text = (
            "OpenCV Coordinate System:\n"
            "X: Right (+), Left (-)\n"
            "Y: Down (+), Up (-)\n"
            "Z: Depth, direction away from camera\n\n"
            "Press 'q' on camera window to exit"
        )
        
        ttk.Label(info_frame, text=info_text, justify="left").grid(row=0, column=0, padx=5, pady=5, sticky="w")
    
    def request_update(self):
        """Request an update for the marker position"""
        self.pending_update = True
    
    def update_timer_callback(self):
        """Timer callback to update marker position at a controlled rate"""
        if self.pending_update and not self.service_busy:
            self.pending_update = False
            self.update_marker_model_pose()
    
    def update_marker_model_pose(self):
        """Update marker model position and pose in Gazebo"""
        self.service_busy = True
        
        # Convert from OpenCV coordinate system to Gazebo coordinate system
        # Gazebo world coordinates: x:forward, y:left, z:up
        # OpenCV camera coordinates: x:right(+), y:down(+), z:away from camera
        
        # Base position of nasa_satellite2 model (world coordinates)
        model_world_pos = self.model_world_pos
        
        # Base orientation of nasa_satellite2 model (world coordinates)
        model_world_angles = self.model_world_angles
        R_world_model = self.euler_to_rotation_matrix(model_world_angles)
        
        # Camera sensor's relative position and orientation to the model
        camera_model_pos = self.camera_model_pos
        camera_model_angles = self.camera_model_angles
        R_model_camera = self.euler_to_rotation_matrix(camera_model_angles)
        
        # Camera's position and orientation in world coordinates
        H_world_model = self.build_homogeneous_matrix(R_world_model, model_world_pos)
        H_model_camera = self.build_homogeneous_matrix(R_model_camera, camera_model_pos)
        H_world_camera = H_world_model @ H_model_camera
        
        # Marker's position and orientation in camera coordinates (OpenCV)
        # OpenCV coordinates: x:right(+), y:down(+), z:depth(+)
        opencv_tvec = np.array([
            self.marker_pose['x'],
            self.marker_pose['y'],
            self.marker_pose['z']
        ]).reshape(3, 1)
        
        # Rotation matrix in OpenCV coordinates
        opencv_angles = (
            self.marker_pose['roll'],
            self.marker_pose['pitch'],
            self.marker_pose['yaw']
        )
        R_opencv = self.euler_to_rotation_matrix(opencv_angles)
        
        # Apply 90-degree rotation around X-axis to align ArUco marker correctly
        R_marker_correction = self.euler_to_rotation_matrix((90, 0, 0))
        R_opencv_corrected = R_opencv @ R_marker_correction
        
        # Transformation matrix: OpenCV camera coordinates -> internal camera coordinates (ROS style)
        # Internal(ROS): x:forward, y:left, z:up
        # OpenCV: x:right, y:down, z:away from camera
        T = np.array([
            [0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0]
        ])
        
        # Coordinate transformation
        R_camera_marker = T @ R_opencv_corrected @ T.T
        t_camera_marker = np.zeros((3, 1))
        t_camera_marker[0, 0] = opencv_tvec[2, 0]  # OpenCV Z -> ROS X
        t_camera_marker[1, 0] = -opencv_tvec[0, 0]  # OpenCV X -> ROS -Y
        t_camera_marker[2, 0] = -opencv_tvec[1, 0]  # OpenCV Y -> ROS -Z
        
        # Homogeneous transformation from camera to marker
        H_camera_marker = self.build_homogeneous_matrix(R_camera_marker, t_camera_marker)
        
        # Marker's position and orientation in world coordinates
        H_world_marker = H_world_camera @ H_camera_marker
        R_world_marker, t_world_marker = self.decompose_homogeneous_matrix(H_world_marker)
        
        # Apply marker model offset (transform to world coordinates)
        offset_world = R_world_marker @ np.array([
            self.marker_model_offset['x'],
            self.marker_model_offset['y'], 
            self.marker_model_offset['z']
        ]).reshape(3, 1)
        
        # Convert to quaternion
        q_world_marker = self.rotation_matrix_to_quaternion(R_world_marker)
        
        # Gazebo model update request
        request = SetEntityPose.Request()
        request.entity.name = f"aruco_marker_{self.marker_id}"
        request.entity.type = request.entity.MODEL
        
        # Set new position with offset applied
        new_pose = Pose()
        new_pose.position.x = float(t_world_marker[0, 0] + offset_world[0, 0])
        new_pose.position.y = float(t_world_marker[1, 0] + offset_world[1, 0])
        new_pose.position.z = float(t_world_marker[2, 0] + offset_world[2, 0])
        
        # Set new orientation
        new_pose.orientation.x = q_world_marker[0]
        new_pose.orientation.y = q_world_marker[1]
        new_pose.orientation.z = q_world_marker[2]
        new_pose.orientation.w = q_world_marker[3]
        
        request.pose = new_pose
        
        # Service call (non-blocking)
        future = self.pose_client.call_async(request)
        future.add_done_callback(self.pose_service_callback)
    
    def pose_service_callback(self, future):
        """Callback for the SetEntityPose service"""
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error("Failed to update marker model position")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")
        finally:
            self.service_busy = False
    
    def image_callback(self, msg):
        """Camera image callback function"""
        try:
            # Store the last received image message for use with auto-adjustment
            self.last_image_msg = msg
            
            # Convert ROS image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Calculate marker's 3D coordinates (marker corners)
            half = self.marker_size / 2
            obj_pts = np.float32([
                [-half, -half, 0],  # Top-left
                [half, -half, 0],   # Top-right
                [half, half, 0],    # Bottom-right
                [-half, half, 0]    # Bottom-left
            ])
            
            # Marker position and orientation in OpenCV coordinates
            tvec = np.array([
                self.marker_pose['x'],
                self.marker_pose['y'],
                self.marker_pose['z']
            ]).reshape(3, 1)
            
            angles = (
                self.marker_pose['roll'],
                self.marker_pose['pitch'],
                self.marker_pose['yaw']
            )
            R = self.euler_to_rotation_matrix(angles)
            rvec, _ = cv2.Rodrigues(R)
            
            # Projection calculation (3D -> 2D)
            img_pts, _ = cv2.projectPoints(obj_pts, rvec, tvec, self.camera_matrix, self.dist_coeffs)
            pts = np.int32(img_pts.reshape(-1, 2))
            
            # Draw marker outline
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
            
            # Display marker ID
            cv2.putText(frame, f"ID: {self.marker_id}", (pts[0][0], pts[0][1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Draw coordinate axes
            cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, length=0.1, thickness=2)
            
            # Display position and rotation information
            info_text = [
                f"Position (m): X:{self.marker_pose['x']:.2f} Y:{self.marker_pose['y']:.2f} Z:{self.marker_pose['z']:.2f}",
                f"Rotation (deg): R:{self.marker_pose['roll']:.1f} P:{self.marker_pose['pitch']:.1f} Y:{self.marker_pose['yaw']:.1f}",
            ]
            
            # Display each line of text
            for i, text in enumerate(info_text):
                cv2.putText(frame, text, (10, 30 + i * 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Show image
            cv2.imshow("ArUco Marker Simulation", frame)
            
            # Press 'q' to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.root.quit()
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")
    
    # --- Utility functions ---
    def euler_to_rotation_matrix(self, angles):
        """Convert Euler angles (RPY, degrees) to rotation matrix"""
        roll, pitch, yaw = np.deg2rad(angles)
        
        # Roll rotation (X-axis)
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        
        # Pitch rotation (Y-axis)
        R_y = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        
        # Yaw rotation (Z-axis)
        R_z = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        # Rotation order: Z -> Y -> X (Yaw -> Pitch -> Roll)
        return R_z @ R_y @ R_x
    
    def rotation_matrix_to_quaternion(self, R):
        """Convert 3x3 rotation matrix to quaternion"""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
        
        return [qx, qy, qz, qw]
    
    def build_homogeneous_matrix(self, R, t):
        """Create 4x4 homogeneous transformation matrix from rotation matrix and translation vector"""
        H = np.eye(4)
        H[0:3, 0:3] = R
        H[0:3, 3] = t.flatten()
        return H
    
    def decompose_homogeneous_matrix(self, H):
        """Extract rotation matrix and translation vector from 4x4 homogeneous transformation matrix"""
        R = H[0:3, 0:3]
        t = H[0:3, 3].reshape(3, 1)
        return R, t

def main(args=None):
    rclpy.init(args=args)
    
    # Create node
    node = ArucoSimulator()
    
    # Set up Tkinter callback to periodically call ROS spin
    def spin_once():
        rclpy.spin_once(node, timeout_sec=0.001)
        node.root.after(10, spin_once)
    
    # Start Tkinter event loop
    node.root.after(10, spin_once)
    node.root.mainloop()
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()