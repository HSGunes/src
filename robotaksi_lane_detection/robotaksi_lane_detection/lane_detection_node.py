#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
import math
from cv_bridge import CvBridge
from collections import deque

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.steering_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.center_offset_pub = self.create_publisher(Float32, '/lane_center_offset', 1)
        self.debug_image_pub = self.create_publisher(Image, '/lane_detection_debug', 1)
        self.brake_command_pub = self.create_publisher(Bool, '/brake_command', 1)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera_rgb/image_raw',
            self.image_callback,
            1
        )
        
        # Laser scan subscription for obstacle detection
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            1
        )
        
        # Obstacle detection parameters
        self.obstacle_distance = float('inf')
        self.obstacle_detected = False
        self.obstacle_detection_distance = 5.0  # 5 meters as requested
        self.front_angle_range = 5.0  # ±5 derece (toplam 10 derece) - genişletildi
        
        # Dynamic vs Static obstacle detection
        self.obstacle_history = deque(maxlen=10)  # Son 10 ölçüm
        self.obstacle_position_threshold = 0.3  # 30cm hareket = dinamik
        self.obstacle_static_count = 0
        self.obstacle_dynamic_count = 0
        self.min_samples_for_classification = 5  # Minimum 5 ölçüm gerekli
        self.is_obstacle_static = None  # None, True, False
        self.obstacle_wait_time = 5.0  # Dinamik engel için maksimum bekleme süresi (saniye)
        self.obstacle_wait_start_time = None
        
        # Smart obstacle avoidance parameters
        self.avoidance_enabled = True
        self.avoidance_steering_gain = 0.8  # Kaçınma için ek direksiyon kazancı
        self.avoidance_speed_reduction = 0.6  # Kaçınma sırasında hız azaltma
        self.safe_distance_for_avoidance = 3.0  # Kaçınma için güvenli mesafe
        
        # Multi-direction obstacle detection
        self.left_side_distance = float('inf')   # Sol taraf mesafe
        self.right_side_distance = float('inf')  # Sağ taraf mesafe
        self.front_distance = float('inf')       # Ön taraf mesafe
        self.avoidance_direction = 0  # -1: sol, 0: düz, +1: sağ
        self.side_check_angle = 25.0  # Yan taraf kontrol açısı (±25°)
        
        # Control publishing timer - publish cmd_vel at fixed rate
        self.control_timer = self.create_timer(0.1, self.publish_control_commands)  # 10 Hz
        
        # Latest control values
        self.latest_steering_angle = 0.0
        self.latest_speed = 0.0
        self.last_control_update = self.get_clock().now()
        self.control_timeout = 1.0  # seconds - stop if no new data
        
        # Lane detection parameters - Classic CV approach
        self.image_height = 480
        self.image_width = 640
        
        # Gaussian Blur parameters
        self.gaussian_kernel_size = 5
        
        # Canny Edge Detection parameters
        self.canny_low_threshold = 50
        self.canny_high_threshold = 150
        
        # Region of Interest parameters (trapezoid)
        self.roi_vertices = None
        
        # Hough Transform parameters
        self.rho = 1                    # Distance resolution in pixels
        self.theta = np.pi/180          # Angular resolution in radians
        self.hough_threshold = 20       # Minimum votes for line detection
        self.min_line_length = 40       # Minimum line length
        self.max_line_gap = 20          # Maximum gap between line segments
        
        # Lane line parameters
        self.left_lane_history = deque(maxlen=10)
        self.right_lane_history = deque(maxlen=10)
        
        # Vehicle control parameters
        self.base_speed = 100.0  # Set to exactly 100 as requested
        self.max_speed = 120.0   # Slightly higher max
        self.min_speed = 50.0    # Reasonable minimum
        self.max_angular_velocity = 1.0
        
        # PID controller parameters
        self.kp = 1.0
        self.ki = 0.05
        self.kd = 0.3
        self.prev_error = 0
        self.integral = 0
        
        # Declare ROS parameters
        self.declare_parameter('gaussian_kernel_size', 5)
        self.declare_parameter('canny_low_threshold', 50)
        self.declare_parameter('canny_high_threshold', 150)
        self.declare_parameter('hough_threshold', 20)
        self.declare_parameter('min_line_length', 40)
        self.declare_parameter('max_line_gap', 20)
        self.declare_parameter('base_speed', 0.8)
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('min_speed', 0.3)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.05)
        self.declare_parameter('kd', 0.3)
        self.declare_parameter('enable_speed_control', True)
        self.declare_parameter('enable_smoothing', True)
        self.declare_parameter('control_rate', 10.0)
        
        # Update control timer with the configured rate
        control_rate = self.get_parameter('control_rate').value
        self.control_timer.destroy()  # Destroy the default timer
        self.control_timer = self.create_timer(1.0 / control_rate, self.publish_control_commands)
        
        self.get_logger().info("Classic Lane Detection Node initialized")
        self.get_logger().info("Using traditional Computer Vision pipeline")
        self.get_logger().info(f"Control publishing rate: {control_rate} Hz")

    def grayscale(self, img):
        """Convert image to grayscale"""
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    def gaussian_blur(self, img, kernel_size):
        """Apply Gaussian blur to reduce noise"""
        return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

    def canny(self, img, low_threshold, high_threshold):
        """Apply Canny edge detection"""
        return cv2.Canny(img, low_threshold, high_threshold)

    def region_of_interest(self, img, vertices):
        """Apply region of interest mask"""
        # Define a blank mask to start with
        mask = np.zeros_like(img)
        
        # Define a 3 channel or 1 channel color to fill the mask with depending on the input image
        if len(img.shape) > 2:
            channel_count = img.shape[2]
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255
            
        # Fill pixels inside the polygon defined by "vertices" with the fill color
        cv2.fillPoly(mask, vertices, ignore_mask_color)
        
        # Return the image only where mask pixels are nonzero
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image

    def get_roi_vertices(self, img_height, img_width):
        """Define the region of interest based on user-provided pixel coordinates"""
        # ROI coordinates based on actual pixel selection from user's camera image
        # Original coordinates: (3,401), (215,234), (396,230), (588,383)
        # Converted to percentage-based for different resolutions
        
        # Bottom vertices (from user's pixel coordinates)
        bottom_left = [int(img_width * 0.005), int(img_height * 0.835)]    # ~(3, 401) for 640x480
        bottom_right = [int(img_width * 0.92), int(img_height * 0.798)]    # ~(588, 383) for 640x480
        
        # Top vertices (from user's pixel coordinates)  
        top_left = [int(img_width * 0.336), int(img_height * 0.488)]       # ~(215, 234) for 640x480
        top_right = [int(img_width * 0.619), int(img_height * 0.479)]      # ~(396, 230) for 640x480
        
        vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
        return vertices

    def hough_lines(self, img, rho, theta, threshold, min_line_len, max_line_gap):
        """Apply Hough transform to detect lines"""
        lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]),
                               minLineLength=min_line_len, maxLineGap=max_line_gap)
        return lines

    def separate_left_right_lines(self, lines, img_width):
        """Separate left and right lane lines based on slope and position"""
        left_lines = []
        right_lines = []
        
        if lines is None:
            return left_lines, right_lines
            
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            # Calculate slope
            if x2 - x1 == 0:  # Avoid division by zero
                continue
            slope = (y2 - y1) / (x2 - x1)
            
            # Filter out lines with very small slopes (nearly horizontal)
            if abs(slope) < 0.4:  # Increased from 0.3 for better filtering
                continue
            
            # Filter out lines that are too steep (likely not lane lines)
            if abs(slope) > 2.5:  # Reduced from 3.0 for stricter filtering
                continue
            
            # Filter lines that are too short or in wrong areas
            line_length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            if line_length < 30:  # Minimum line length
                continue
                
            # Calculate the center point of the line
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            
            # Only consider lines in the bottom 2/3 of the image (road area)
            if center_y < img_width * 0.33:  # Skip lines in top 1/3
                continue
            
            # More robust classification based on slope, position, and line center
            if slope < -0.4:  # Negative slope - potential left line
                # Left line should be in the left half or center-left area
                if center_x < img_width * 0.7:  # Allow some overlap into center
                    left_lines.append(line[0])
            elif slope > 0.4:  # Positive slope - potential right line  
                # Right line should be in the right half or center-right area
                if center_x > img_width * 0.3:  # Allow some overlap into center
                    right_lines.append(line[0])
                
        return left_lines, right_lines

    def average_slope_intercept(self, lines):
        """Calculate average slope and intercept for a set of lines"""
        if not lines:
            return None
            
        slopes = []
        intercepts = []
        
        for line in lines:
            x1, y1, x2, y2 = line
            if x2 - x1 == 0:  # Avoid division by zero
                continue
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - slope * x1
            slopes.append(slope)
            intercepts.append(intercept)
        
        if not slopes:
            return None
            
        avg_slope = np.mean(slopes)
        avg_intercept = np.mean(intercepts)
        
        return avg_slope, avg_intercept

    def make_coordinates(self, img_shape, line_parameters):
        """Create line coordinates from slope and intercept"""
        if line_parameters is None:
            return None
            
        slope, intercept = line_parameters
        y1 = img_shape[0]  # Bottom of the image
        y2 = int(y1 * 0.6)  # Top of ROI
        
        if slope == 0:  # Avoid division by zero
            return None
            
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)
        
        return np.array([x1, y1, x2, y2])

    def smooth_lines(self, new_line, line_history):
        """Apply temporal smoothing to detected lines"""
        if not self.get_parameter('enable_smoothing').value:
            return new_line
            
        if new_line is not None:
            line_history.append(new_line)
        
        if len(line_history) == 0:
            return None
            
        # Calculate weighted average (more recent lines have higher weight)
        weights = np.linspace(0.5, 1.0, len(line_history))
        
        avg_x1 = int(np.average([line[0] for line in line_history], weights=weights))
        avg_y1 = int(np.average([line[1] for line in line_history], weights=weights))
        avg_x2 = int(np.average([line[2] for line in line_history], weights=weights))
        avg_y2 = int(np.average([line[3] for line in line_history], weights=weights))
        
        return np.array([avg_x1, avg_y1, avg_x2, avg_y2])

    def draw_lines(self, img, lines, color=[0, 255, 0], thickness=8):
        """Draw lines on image"""
        line_img = np.zeros_like(img)
        
        if lines is not None:
            for line in lines:
                if line is not None:
                    x1, y1, x2, y2 = line
                    cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)
        
        return line_img

    def weighted_img(self, img, initial_img, α=0.8, β=1., γ=0.):
        """Combine two images with weights"""
        return cv2.addWeighted(initial_img, α, img, β, γ)

    def calculate_lane_center(self, left_line, right_line, img_width):
        """Calculate the center of the lane"""
        if left_line is None and right_line is None:
            return img_width // 2, 0  # Return image center if no lines detected
        
        # Use bottom of the image (y = img_height) to calculate center
        img_height = self.image_height
        
        if left_line is not None and right_line is not None:
            # Both lines detected - this is the most reliable case
            left_x = left_line[0]  # x1 of left line (bottom point)
            right_x = right_line[0]  # x1 of right line (bottom point)
            
            # Validate that the lines make sense (left should be left of right)
            if left_x >= right_x:
                self.get_logger().warn(f"Invalid line positions: left_x={left_x}, right_x={right_x}")
                # Try to fix by swapping if they're close
                if abs(left_x - right_x) < 50:
                    left_x, right_x = right_x, left_x
                else:
                    # Lines are too confused, fall back to image center
                    return 0, 0.1
            
            lane_center = (left_x + right_x) // 2
            confidence = 1.0
            
        elif left_line is not None:
            # Only left line detected
            left_x = left_line[0]
            # Estimate right line position based on typical lane width
            # Adjusted lane width based on camera perspective and typical lane widths
            estimated_lane_width = min(280, img_width * 0.4)  # More conservative estimate
            estimated_right_x = left_x + estimated_lane_width
            
            # Make sure the estimated right line doesn't go off screen
            if estimated_right_x >= img_width:
                estimated_right_x = img_width - 20
            
            lane_center = (left_x + estimated_right_x) // 2
            confidence = 0.6  # Lower confidence when estimating
            
        elif right_line is not None:
            # Only right line detected
            right_x = right_line[0]
            # Estimate left line position
            estimated_lane_width = min(280, img_width * 0.4)  # More conservative estimate
            estimated_left_x = right_x - estimated_lane_width
            
            # Make sure the estimated left line doesn't go off screen
            if estimated_left_x < 0:
                estimated_left_x = 20
            
            lane_center = (estimated_left_x + right_x) // 2
            confidence = 0.6  # Lower confidence when estimating
            
        else:
            # This shouldn't happen given the first check, but just in case
            lane_center = img_width // 2
            confidence = 0.0
        
        # Calculate offset from image center
        image_center = img_width // 2
        offset = lane_center - image_center
        
        # Add bounds checking for offset
        max_reasonable_offset = img_width // 3  # Don't allow offset larger than 1/3 of image width
        if abs(offset) > max_reasonable_offset:
            self.get_logger().warn(f"Offset too large: {offset}px, clamping to {max_reasonable_offset}px")
            offset = np.clip(offset, -max_reasonable_offset, max_reasonable_offset)
            confidence *= 0.5  # Reduce confidence for clamped values
        
        return offset, confidence

    def calculate_steering_angle(self, offset, img_width, confidence):
        """Calculate steering angle using PID controller"""
        # Get parameters
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        
        # Normalize offset to [-1, 1] range
        # Use a more conservative normalization to avoid extreme values
        max_offset = img_width * 0.4  # Use 40% of image width as max expected offset
        normalized_offset = np.clip(offset / max_offset, -1.0, 1.0)
        
        # Apply confidence weighting - reduce response when confidence is low
        error = normalized_offset * confidence
        
        # Add deadband to reduce jitter for small errors
        deadband = 0.05
        if abs(error) < deadband:
            error = 0.0
        
        # PID controller
        self.integral += error
        # Prevent integral windup with more conservative limits
        self.integral = np.clip(self.integral, -0.5, 0.5)
        
        derivative = error - self.prev_error
        
        # Calculate steering angle
        steering_angle = kp * error + ki * self.integral + kd * derivative
        
        # IMPORTANT: Invert the sign for correct steering direction
        # Positive offset (vehicle right of lane center) -> Negative angular.z (right turn)
        # Negative offset (vehicle left of lane center) -> Positive angular.z (left turn)
        steering_angle = -steering_angle
        
        # Apply non-linear scaling for better control
        # Small errors get proportionally smaller response
        if abs(steering_angle) < 0.2:
            steering_angle *= 0.7  # Reduce small corrections
        
        # Limit steering angle
        max_angular = self.get_parameter('max_angular_velocity').value
        steering_angle = np.clip(steering_angle, -max_angular, max_angular)
        
        self.prev_error = error
        
        return steering_angle

    def calculate_speed(self, offset, confidence):
        """Calculate adaptive speed based on lane position and confidence"""
        if not self.get_parameter('enable_speed_control').value:
            return self.get_parameter('base_speed').value
        
        base_speed = self.get_parameter('base_speed').value
        max_speed = self.get_parameter('max_speed').value
        min_speed = self.get_parameter('min_speed').value
        
        # Use normalized offset for speed calculation
        abs_offset = abs(offset)
        
        # More granular speed control based on offset
        if abs_offset > 120:  # Very sharp turn
            speed_factor = 0.4
        elif abs_offset > 80:  # Sharp turn
            speed_factor = 0.6
        elif abs_offset > 40:  # Moderate turn
            speed_factor = 0.8
        elif abs_offset > 20:  # Slight curve
            speed_factor = 0.9
        else:  # Straight or very slight curve
            speed_factor = 1.0
        
        # Apply confidence factor - reduce speed when confidence is low
        confidence_factor = max(0.5, confidence)  # Don't go below 50% speed due to confidence
        
        # Calculate final speed
        target_speed = base_speed * speed_factor * confidence_factor
        
        # Ensure speed is within bounds
        target_speed = np.clip(target_speed, min_speed, max_speed)
        
        return target_speed

    def create_debug_image(self, original_img, left_line, right_line, offset, confidence):
        """Create debug visualization"""
        # Start with original image
        debug_img = original_img.copy()
        
        # ROI removed - no longer drawing ROI polygon
        # roi_vertices = self.get_roi_vertices(debug_img.shape[0], debug_img.shape[1])
        # cv2.polylines(debug_img, roi_vertices, True, (255, 255, 0), 2)
        
        # Draw lane lines with better visualization
        if left_line is not None:
            x1, y1, x2, y2 = left_line
            cv2.line(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 8)
            cv2.putText(debug_img, "LEFT", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            # Mark the bottom point used for calculation
            cv2.circle(debug_img, (x1, y1), 8, (0, 255, 255), -1)
        
        if right_line is not None:
            x1, y1, x2, y2 = right_line
            cv2.line(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 8)
            cv2.putText(debug_img, "RIGHT", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            # Mark the bottom point used for calculation
            cv2.circle(debug_img, (x1, y1), 8, (0, 255, 255), -1)
        
        # Draw center lines with better visualization
        img_center = debug_img.shape[1] // 2
        lane_center = img_center + offset
        
        # Image center line (red)
        cv2.line(debug_img, (img_center, debug_img.shape[0]), 
                (img_center, int(debug_img.shape[0] * 0.6)), (0, 0, 255), 3)
        cv2.putText(debug_img, "IMG CENTER", (img_center - 50, debug_img.shape[0] - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # Lane center line (blue)
        cv2.line(debug_img, (int(lane_center), debug_img.shape[0]), 
                (int(lane_center), int(debug_img.shape[0] * 0.6)), (255, 0, 0), 3)
        cv2.putText(debug_img, "LANE CENTER", (int(lane_center) - 50, debug_img.shape[0] - 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        # Draw offset arrow
        if abs(offset) > 5:  # Only draw arrow if offset is significant
            arrow_start = (img_center, debug_img.shape[0] - 50)
            arrow_end = (int(lane_center), debug_img.shape[0] - 50)
            cv2.arrowedLine(debug_img, arrow_start, arrow_end, (255, 255, 0), 3, tipLength=0.3)
        
        # Add comprehensive text information
        y_pos = 30
        cv2.putText(debug_img, f"Offset: {offset:.1f}px", (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        y_pos += 30
        cv2.putText(debug_img, f"Confidence: {confidence:.2f}", (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        y_pos += 30
        
        lane_status = "Both" if left_line is not None and right_line is not None else \
                     ("Left Only" if left_line is not None else ("Right Only" if right_line is not None else "None"))
        cv2.putText(debug_img, f"Lanes: {lane_status}", (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        y_pos += 30
        
        # Add normalized offset for reference
        max_offset = debug_img.shape[1] * 0.4
        normalized_offset = np.clip(offset / max_offset, -1.0, 1.0)
        cv2.putText(debug_img, f"Norm. Offset: {normalized_offset:.2f}", (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        y_pos += 30
        
        # Add steering direction indicator with obstacle avoidance info
        if abs(offset) > 5 or self.obstacle_detected:
            if self.obstacle_detected:
                obstacle_type = self.classify_obstacle()
                if obstacle_type == "static" or self.is_obstacle_static == True:
                    if self.avoidance_direction == -1:
                        direction_text = f"STATIC OBSTACLE - AVOIDING LEFT (L:{self.left_side_distance:.1f}m)"
                        direction_color = (255, 0, 0)  # Blue for left avoidance
                    elif self.avoidance_direction == 1:
                        direction_text = f"STATIC OBSTACLE - AVOIDING RIGHT (R:{self.right_side_distance:.1f}m)"
                        direction_color = (0, 0, 255)  # Red for right avoidance
                    else:
                        direction_text = f"STATIC OBSTACLE - NO ESCAPE ROUTE (L:{self.left_side_distance:.1f}m R:{self.right_side_distance:.1f}m)"
                        direction_color = (0, 0, 128)  # Dark red for blocked
                elif obstacle_type == "dynamic" or self.is_obstacle_static == False:
                    direction_text = f"DYNAMIC OBSTACLE - WAITING (dist: {self.obstacle_distance:.1f}m)"
                    direction_color = (0, 255, 255)  # Yellow for dynamic
                else:
                    direction_text = f"OBSTACLE ANALYZING (dist: {self.obstacle_distance:.1f}m)"
                    direction_color = (128, 128, 128)  # Gray for analyzing
            else:
                # Normal lane following
                if offset > 0:
                    direction_text = "Need RIGHT turn"
                    direction_color = (0, 165, 255)  # Orange
                else:
                    direction_text = "Need LEFT turn"
                    direction_color = (255, 165, 0)  # Blue
            
            cv2.putText(debug_img, direction_text, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, direction_color, 2)
        
        return debug_img

    def scan_callback(self, msg):
        """Process laser scan data for obstacle detection with smart multi-direction avoidance"""
        try:
            if not msg.ranges:
                return
            
            # Çoklu yön engel tespiti
            front_distances = []
            left_side_distances = []
            right_side_distances = []
            
            for i, distance in enumerate(msg.ranges):
                if msg.range_min <= distance <= msg.range_max:
                    angle = msg.angle_min + i * msg.angle_increment
                    angle_deg = math.degrees(angle)
                    
                    # Açıyı normalize et (-180 to 180)
                    while angle_deg > 180:
                        angle_deg -= 360
                    while angle_deg < -180:
                        angle_deg += 360
                    
                    # Ön bölge kontrolü (175° to 185° = tam ön ±5°)
                    # 180° = tam ön, 175° = sağ ön, -175° = sol ön (185° = -175°)
                    if (175 <= angle_deg <= 180) or (-180 <= angle_deg <= -175):
                        front_distances.append(distance)
                    
                    # Sol taraf kontrolü (150° to 170°)
                    elif 150 <= angle_deg <= 170:
                        left_side_distances.append(distance)
                    
                    # Sağ taraf kontrolü (-170° to -150°)
                    elif -170 <= angle_deg <= -150:
                        right_side_distances.append(distance)
            # Mesafeleri hesapla
            current_time = self.get_clock().now()
            
            self.front_distance = min(front_distances) if front_distances else float('inf')
            self.left_side_distance = min(left_side_distances) if left_side_distances else float('inf')
            self.right_side_distance = min(right_side_distances) if right_side_distances else float('inf')
            
            # Ana engel tespiti (ön bölge)
            if self.front_distance <= self.obstacle_detection_distance:
                self.obstacle_distance = self.front_distance
                
                # Engel mesafesini geçmişe kaydet
                self.obstacle_history.append((current_time, self.front_distance))
                
                # Statik/Dinamik sınıflandırma
                obstacle_type = self.classify_obstacle()
                
                # Kaçınma yönünü belirle (sadece statik engeller için)
                if obstacle_type == "static" or self.is_obstacle_static == True:
                    self.determine_avoidance_direction()
                
                if not self.obstacle_detected:
                    self.obstacle_detected = True
                    if obstacle_type == "static":
                        direction_text = self.get_avoidance_direction_text()
                        self.get_logger().warn(f"🚨 STATİK ENGEL ALGILANDI! Mesafe: {self.front_distance:.2f}m - {direction_text}")
                    elif obstacle_type == "dynamic":
                        self.get_logger().warn(f"🚨 DİNAMİK ENGEL ALGILANDI! Mesafe: {self.front_distance:.2f}m - GEÇENE KADAR BEKLİYOR")
                        self.obstacle_wait_start_time = current_time
                    else:
                        self.get_logger().warn(f"🚨 ENGEL ALGILANDI! Mesafe: {self.front_distance:.2f}m - ANALİZ EDİLİYOR...")
                
                # Dinamik engel için geçme kontrolü
                if obstacle_type == "dynamic" and self.obstacle_wait_start_time is not None:
                    wait_duration = (current_time - self.obstacle_wait_start_time).nanoseconds / 1e9
                    if wait_duration > self.obstacle_wait_time:
                        self.get_logger().warn(f"⚠️ DİNAMİK ENGEL {self.obstacle_wait_time}s BOYUNCA GEÇMEDİ - KAÇINMA MODUNA GEÇİLİYOR")
                        # Çok uzun süre bekledik, statik gibi davran
                        self.is_obstacle_static = True
                        self.determine_avoidance_direction()
            
            else:
                # Engel kaldırıldığında
                if self.obstacle_detected:
                    self.obstacle_detected = False
                    self.obstacle_wait_start_time = None
                    self.avoidance_direction = 0  # Kaçınma yönünü sıfırla
                    self.reset_obstacle_classification()
                    self.get_logger().info(f"✅ ENGEL KALDIRILDI! Mesafe: {self.front_distance:.2f}m - NORMAL HIZ")
                
                # Mesafeleri güncelle
                self.obstacle_distance = float('inf')
            
        except Exception as e:
            self.get_logger().error(f"Error in scan callback: {str(e)}")

    def classify_obstacle(self):
        """Engeli statik veya dinamik olarak sınıflandır"""
        if len(self.obstacle_history) < self.min_samples_for_classification:
            return "analyzing"  # Henüz yeterli veri yok
        
        # Son ölçümlerdeki mesafe değişimlerini analiz et
        distances = [entry[1] for entry in list(self.obstacle_history)[-self.min_samples_for_classification:]]
        
        # Mesafe değişiminin standart sapmasını hesapla
        distance_changes = []
        for i in range(1, len(distances)):
            distance_changes.append(abs(distances[i] - distances[i-1]))
        
        if not distance_changes:
            return "analyzing"
        
        avg_change = np.mean(distance_changes)
        max_change = max(distance_changes)
        
        # Sınıflandırma kriterleri
        if max_change > self.obstacle_position_threshold:
            # Büyük değişim = dinamik engel (hareket ediyor)
            self.obstacle_dynamic_count += 1
            self.obstacle_static_count = max(0, self.obstacle_static_count - 1)
            
            if self.obstacle_dynamic_count >= 3:
                self.is_obstacle_static = False
                return "dynamic"
        else:
            # Küçük değişim = statik engel (sabit)
            self.obstacle_static_count += 1
            self.obstacle_dynamic_count = max(0, self.obstacle_dynamic_count - 1)
            
            if self.obstacle_static_count >= 3:
                self.is_obstacle_static = True
                return "static"
        
        # Henüz kesin karar verilemedi
        return "analyzing"

    def reset_obstacle_classification(self):
        """Engel sınıflandırmasını sıfırla"""
        self.obstacle_history.clear()
        self.obstacle_static_count = 0
        self.obstacle_dynamic_count = 0
        self.is_obstacle_static = None

    def determine_avoidance_direction(self):
        """Engelden kaçmak için en iyi yönü belirle"""
        # Minimum güvenli mesafe
        min_safe_distance = 2.0
        
        # Sol ve sağ tarafın güvenli olup olmadığını kontrol et
        left_safe = self.left_side_distance > min_safe_distance
        right_safe = self.right_side_distance > min_safe_distance
        
        if left_safe and right_safe:
            # Her iki taraf da güvenli - daha geniş olan tarafa kaç
            if self.left_side_distance > self.right_side_distance:
                self.avoidance_direction = -1  # Sol
            else:
                self.avoidance_direction = 1   # Sağ
        elif left_safe and not right_safe:
            # Sadece sol güvenli
            self.avoidance_direction = -1  # Sol
        elif right_safe and not left_safe:
            # Sadece sağ güvenli
            self.avoidance_direction = 1   # Sağ
        else:
            # Hiçbir taraf güvenli değil - dur
            self.avoidance_direction = 0   # Dur
        
        self.get_logger().debug(f"Avoidance analysis: Left={self.left_side_distance:.1f}m, Right={self.right_side_distance:.1f}m, Direction={self.avoidance_direction}")

    def get_avoidance_direction_text(self):
        """Kaçınma yönü metnini döndür"""
        if self.avoidance_direction == -1:
            return f"SOLA KAÇINIYOR (L:{self.left_side_distance:.1f}m > R:{self.right_side_distance:.1f}m)"
        elif self.avoidance_direction == 1:
            return f"SAĞA KAÇINIYOR (R:{self.right_side_distance:.1f}m > L:{self.left_side_distance:.1f}m)"
        else:
            return f"KAÇIŞ YÖN YOK - DURMA (L:{self.left_side_distance:.1f}m, R:{self.right_side_distance:.1f}m)"

    def lane_detection_pipeline(self, image):
        """Main lane detection pipeline"""
        # Store original image dimensions
        img_height, img_width = image.shape[:2]
        self.image_height = img_height
        self.image_width = img_width
        
        # Step 1: Convert to grayscale
        gray = self.grayscale(image)
        
        # Step 2: Apply Gaussian blur
        kernel_size = self.get_parameter('gaussian_kernel_size').value
        blur = self.gaussian_blur(gray, kernel_size)
        
        # Step 3: Apply Canny edge detection
        low_threshold = self.get_parameter('canny_low_threshold').value
        high_threshold = self.get_parameter('canny_high_threshold').value
        edges = self.canny(blur, low_threshold, high_threshold)
        
        # Step 4: ROI removed - use full image for lane detection
        # masked_edges = self.region_of_interest(edges, roi_vertices)
        masked_edges = edges  # Use full edge image
        
        # Step 5: Apply Hough transform
        rho = self.rho
        theta = self.theta
        threshold = self.get_parameter('hough_threshold').value
        min_line_len = self.get_parameter('min_line_length').value
        max_line_gap = self.get_parameter('max_line_gap').value
        
        lines = self.hough_lines(masked_edges, rho, theta, threshold, min_line_len, max_line_gap)
        
        # Step 6: Separate left and right lines
        left_lines, right_lines = self.separate_left_right_lines(lines, img_width)
        
        # Step 7: Average lines to get single left and right lane lines
        left_lane = self.average_slope_intercept(left_lines)
        right_lane = self.average_slope_intercept(right_lines)
        
        # Step 8: Make coordinates for drawing
        left_line = self.make_coordinates(image.shape, left_lane)
        right_line = self.make_coordinates(image.shape, right_lane)
        
        # Step 9: Apply temporal smoothing
        left_line = self.smooth_lines(left_line, self.left_lane_history)
        right_line = self.smooth_lines(right_line, self.right_lane_history)
        
        return left_line, right_line

    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            if cv_image is None or cv_image.size == 0:
                self.get_logger().warn("Received empty or invalid image")
                return
            
            # Run lane detection pipeline
            left_line, right_line = self.lane_detection_pipeline(cv_image)
            
            # Calculate lane center and offset
            offset, confidence = self.calculate_lane_center(left_line, right_line, cv_image.shape[1])
            
            # Calculate steering angle with obstacle avoidance
            base_steering_angle = self.calculate_steering_angle(offset, cv_image.shape[1], confidence)
            
            # Apply obstacle avoidance if needed
            steering_angle = base_steering_angle  # Her zaman normal direksiyon
            
            # Calculate speed - sadece engel varsa dur
            if self.obstacle_detected:
                # Engel var - dur
                speed = 0.0
                brake_command = True
                self.get_logger().debug(f"Obstacle detected - stopping")
            else:
                # Yol açık - normal hız
                speed = 100.0
                brake_command = False
            
            # Update latest control values
            self.latest_steering_angle = steering_angle
            self.latest_speed = speed
            self.last_control_update = self.get_clock().now()
            
            # Publish brake command
            brake_msg = Bool()
            brake_msg.data = brake_command
            self.brake_command_pub.publish(brake_msg)
            
            # Publish lane center offset (this can be published at camera rate)
            offset_msg = Float32()
            offset_msg.data = float(offset)
            self.center_offset_pub.publish(offset_msg)
            
            # Create and publish debug image
            debug_image = self.create_debug_image(cv_image, left_line, right_line, offset, confidence)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            self.debug_image_pub.publish(debug_msg)
            
            # Enhanced logging with obstacle type information
            lane_count = sum([1 for line in [left_line, right_line] if line is not None])
            
            if self.obstacle_distance != float('inf'):
                obstacle_type = self.classify_obstacle() if self.obstacle_detected else "detected"
                obstacle_status = f"Obstacle: {obstacle_type.upper()} at {self.obstacle_distance:.1f}m"
            else:
                obstacle_status = "No obstacle"
                
            self.get_logger().info(
                f"Lanes: {lane_count}/2, Offset: {offset:.1f}px, "
                f"Steering: {steering_angle:.3f}rad/s, Speed: {speed:.2f}m/s, "
                f"Confidence: {confidence:.2f}, {obstacle_status}, "
                f"Brake: {'ON' if brake_command else 'OFF'}"
            )
            
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {str(e)}")
            import traceback
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")

    def publish_control_commands(self):
        """Publish control commands at a fixed rate to avoid overwhelming motors"""
        current_time = self.get_clock().now()
        time_since_update = (current_time - self.last_control_update).nanoseconds / 1e9
        
        if time_since_update > self.control_timeout:
            # No recent lane detection data - stop the vehicle safely
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.steering_pub.publish(twist_msg)
            
            # Log warning but throttle it to avoid spam
            self.get_logger().warn(
                f"Lane detection timeout ({time_since_update:.1f}s), stopping vehicle",
                throttle_duration_sec=2.0
            )
            return
        
        # Publish the latest control commands
        twist_msg = Twist()
        twist_msg.linear.x = self.latest_speed
        twist_msg.angular.z = self.latest_steering_angle
        self.steering_pub.publish(twist_msg)
        
        # Log control commands at reduced frequency
        self.get_logger().debug(
            f"CMD_VEL: Linear: {self.latest_speed:.2f} m/s, "
            f"Angular: {self.latest_steering_angle:.3f} rad/s ({self.latest_steering_angle*180/3.14159:.1f}°)"
        )

def main(args=None):
    rclpy.init(args=args)
    
    lane_detector = LaneDetectionNode()
    
    try:
        rclpy.spin(lane_detector)
    except KeyboardInterrupt:
        pass
    finally:
        lane_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 