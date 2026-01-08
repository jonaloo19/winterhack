#!/usr/bin/env python3
"""
Multi Color Detector with Depth - Detect multiple colors and return 3D position information
Fixed to display English text only
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import yaml
import os
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32MultiArray, Header
from cv_bridge import CvBridge, CvBridgeError
from ament_index_python.packages import get_package_share_directory
import threading
from geometry_msgs.msg import PointStamped, Point
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException, Buffer, TransformListener
import json


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        
        # Load color configuration
        self.load_color_config()
        
        # Create CV bridge
        self.bridge = CvBridge()
        
        # Store latest depth and RGB images
        self.latest_depth_image = None
        self.latest_rgb_image = None
        self.latest_depth_header = None
        self.latest_rgb_header = None
        self.image_lock = threading.Lock()
        
        # Subscribe to RGB camera image
        self.rgb_subscription = self.create_subscription(
            Image,
            '/depth_cam/depth_cam',  # RGB image topic
            self.rgb_image_callback,
            10
        )
        
        # Subscribe to depth image
        self.depth_subscription = self.create_subscription(
            Image,
            '/depth_cam/depth_image',  # Depth image topic
            self.depth_image_callback,
            10
        )
        
        # TF2 related
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publish processed image (with bounding boxes)
        self.image_pub = self.create_publisher(
            Image,
            '/color_detection/debug_image',
            10
        )
        
        # Publish detected color names
        self.color_names_pub = self.create_publisher(
            String,
            '/color_detection/color_names',
            10
        )
        
        # Publish detailed detection info (JSON format)
        self.detection_info_pub = self.create_publisher(
            String,
            '/color_detection/detection_info',
            10
        )
        
        # Publish position with depth
        self.position_pub = self.create_publisher(
            Float32MultiArray,
            '/color_detection/positions_with_depth',
            10
        )
        
        # Publish 3D points
        self.point3d_pub = self.create_publisher(
            PointStamped,
            '/color_detection/object_3d_points',
            10
        )
        
        # Camera intrinsics (need to calibrate for your camera)
        self.camera_info = {
            'fx': 554.25,    # Focal length x (pixels)
            'fy': 554.25,    # Focal length y (pixels)
            'cx': 320.0,     # Principal point x (pixels)
            'cy': 200.0,     # Principal point y (pixels)
            'width': 640,
            'height': 400,
            'scale_factor': 0.001  # Depth scale factor (mm to meters if needed)
        }
        self.declare_parameter("camera_frame_id", "depth_cam_link")
        
        self.get_logger().info('Multi Color Detector with Depth Started')
        self.get_logger().info(f'Supported colors: {list(self.colors.keys())}')
        
        # Create debug window
        self.declare_parameter("show_debug_window", False)
        self.show_debug_window = bool(self.get_parameter("show_debug_window").value)
        self.debug_window_name = "Color Detection with Depth"
        if self.show_debug_window:
            cv2.namedWindow(self.debug_window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.debug_window_name, 800, 600)

    def _normalize_frame_id(self, header):
        """Return a header with a TF-valid frame id."""
        if header is None:
            return None
        if header.frame_id.startswith("robot/"):
            return Header(
                stamp=header.stamp,
                frame_id=str(self.get_parameter("camera_frame_id").value),
            )
        return header
    
    def load_color_config(self):
        """Load color information from config file"""
        try:
            # Get package path
            pkg_dir = get_package_share_directory('winterhack')
            config_path = os.path.join(pkg_dir, 'config/colors.yaml')
            
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            colors_config = config.get('colors', {})
            self.detection_params = config.get('detection', {})
            
            # Preprocess color config
            self.colors = {}
            self.color_names = {}
            
            for color_id, color_info in colors_config.items():
                # Chinese to English mapping
                chinese_to_english = {
                    '红色': 'RED',
                    '绿色': 'GREEN', 
                    '蓝色': 'BLUE',
                    '黄色': 'YELLOW',
                    '红': 'RED',
                    '绿': 'GREEN',
                    '蓝': 'BLUE',
                    '黄': 'YELLOW'
                }
                
                original_name = color_info.get('name', color_id)
                if original_name in chinese_to_english:
                    color_info['name'] = chinese_to_english[original_name]
                else:
                    color_info['name'] = original_name.upper()
                
                self.color_names[color_id] = color_info['name']
                bgr_color = color_info.get('bgr_color', [0, 0, 255])
                
                # Process HSV ranges
                hsv_ranges = []
                if 'lower1' in color_info and 'upper1' in color_info:
                    lower1 = np.array(color_info['lower1'], dtype=np.uint8)
                    upper1 = np.array(color_info['upper1'], dtype=np.uint8)
                    hsv_ranges.append((lower1, upper1))
                    
                    if 'lower2' in color_info and 'upper2' in color_info:
                        lower2 = np.array(color_info['lower2'], dtype=np.uint8)
                        upper2 = np.array(color_info['upper2'], dtype=np.uint8)
                        hsv_ranges.append((lower2, upper2))
                elif 'lower' in color_info and 'upper' in color_info:
                    lower = np.array(color_info['lower'], dtype=np.uint8)
                    upper = np.array(color_info['upper'], dtype=np.uint8)
                    hsv_ranges.append((lower, upper))
                
                self.colors[color_id] = {
                    'name': self.color_names[color_id],
                    'bgr_color': tuple(bgr_color),
                    'hsv_ranges': hsv_ranges
                }
            
            self.get_logger().info(f'Loaded {len(self.colors)} color configurations')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load color config: {e}')
            # Default configuration
            self.colors = {
                'red': {
                    'name': 'RED',
                    'bgr_color': (0, 0, 255),
                    'hsv_ranges': [
                        (np.array([0, 100, 100]), np.array([10, 255, 255])),
                        (np.array([160, 100, 100]), np.array([180, 255, 255]))
                    ]
                },
                'green': {
                    'name': 'GREEN',
                    'bgr_color': (0, 255, 0),
                    'hsv_ranges': [
                        (np.array([40, 100, 100]), np.array([80, 255, 255]))
                    ]
                },
                'blue': {
                    'name': 'BLUE',
                    'bgr_color': (255, 0, 0),
                    'hsv_ranges': [
                        (np.array([100, 100, 100]), np.array([130, 255, 255]))
                    ]
                }
            }
            self.detection_params = {
                'min_area': 500,
                'blur_size': 5,
                'show_mask': True
            }
    
    def depth_image_callback(self, msg):
        """Depth image callback function"""
        try:
            # Convert depth image to OpenCV format
            # Note: Depth images are usually 32FC1 format (32-bit float single channel)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            
            with self.image_lock:
                self.latest_depth_image = depth_image
                self.latest_depth_header = msg.header
                
        except CvBridgeError as e:
            self.get_logger().error(f'Depth image conversion error: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Depth image processing error: {str(e)}')
    
    def rgb_image_callback(self, msg):
        """RGB image callback function"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            with self.image_lock:
                self.latest_rgb_image = cv_image
                self.latest_rgb_header = msg.header
            
            # Get corresponding depth image
            depth_image = None
            depth_header = None
            rgb_header = msg.header
            
            with self.image_lock:
                if self.latest_depth_image is not None:
                    depth_image = self.latest_depth_image.copy()
                    depth_header = self.latest_depth_header
            
            # Process image, detect all colors
            debug_image, detection_info = self.process_image_with_depth(
                cv_image, depth_image, rgb_header, depth_header
            )

            if detection_info and depth_image is not None:
                valid_depths = depth_image[~np.isnan(depth_image) & (depth_image > 0)]
                if len(valid_depths) > 0:
                    self.get_logger().info(
                        f'Depth stats: min={np.min(valid_depths):.3f}m, '
                        f'max={np.max(valid_depths):.3f}m, '
                        f'avg={np.mean(valid_depths):.3f}m',
                        throttle_duration_sec=2.0
                    )
            
            # Publish processed image
            if debug_image is not None:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                debug_msg.header = self._normalize_frame_id(msg.header)
                self.image_pub.publish(debug_msg)
            
            # Publish detection info
            if detection_info:
                header = depth_header if depth_header is not None else rgb_header
                self.publish_detection_info(detection_info, header)
            
            # Show debug window
            if self.show_debug_window:
                cv2.imshow(self.debug_window_name, debug_image)
                cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f'Image processing error: {str(e)}')
    
    def pixel_to_3d(self, u, v, depth_value):
        """Convert pixel coordinates to 3D coordinates (camera frame)"""
        if depth_value <= 0 or np.isnan(depth_value):
            return None
        
        # Using pinhole camera model
        # X = (u - cx) * Z / fx
        # Y = (v - cy) * Z / fy
        # Z = depth_value
        
        fx = self.camera_info['fx']
        fy = self.camera_info['fy']
        cx = self.camera_info['cx']
        cy = self.camera_info['cy']
        
        z = depth_value
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        
        return (x, y, z)
    
    def process_image_with_depth(self, rgb_image, depth_image, rgb_header, depth_header):
        """Process image, detect all colors, and get depth information"""
        # Copy original image for drawing
        debug_image = rgb_image.copy()

        # Draw a small cross at the image center for reference
        self.draw_center_cross(debug_image)
        
        # Convert to HSV color space
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        
        # Blur to reduce noise
        blur_size = self.detection_params.get('blur_size', 5)
        if blur_size > 0:
            hsv = cv2.GaussianBlur(hsv, (blur_size, blur_size), 0)
        
        # Store all detection results
        all_detections = []
        min_area = self.detection_params.get('min_area', 500)
        
        # Detect each color
        for color_id, color_info in self.colors.items():
            color_name = color_info['name']
            bgr_color = color_info['bgr_color']
            hsv_ranges = color_info['hsv_ranges']
            frame_h, frame_w = rgb_image.shape[:2]
            frame_cx = frame_w // 2
            frame_cy = frame_h // 2
            
            # Merge multiple HSV ranges
            mask = None
            for lower, upper in hsv_ranges:
                range_mask = cv2.inRange(hsv, lower, upper)
                if mask is None:
                    mask = range_mask
                else:
                    mask = cv2.bitwise_or(mask, range_mask)
            
            if mask is not None:
                # Morphological operations to remove noise
                kernel = np.ones((5, 5), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                
                # Find contours
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                # Process each detected region
                for i, contour in enumerate(contours):
                    area = cv2.contourArea(contour)
                    
                    if area >= min_area:
                        # Calculate bounding box
                        x, y, w, h = cv2.boundingRect(contour)
                        center_x = x + w // 2
                        center_y = y + h // 2
                        
                        # Calculate aspect ratio
                        aspect_ratio = w / h if h > 0 else 0
                        
                        # Initialize depth related variables
                        depth_value = None
                        point_3d = None
                        distance = None
                        
                        # If depth image exists, get depth information
                        if depth_image is not None:
                            # Ensure coordinates are within image bounds
                            if (0 <= center_y < depth_image.shape[0] and 
                                0 <= center_x < depth_image.shape[1]):
                                
                                # Get depth value at center point
                                depth_value = depth_image[center_y, center_x]
                                
                                # If center depth is invalid, try sampling in the area
                                if np.isnan(depth_value) or depth_value <= 0:
                                    # Sample multiple points in bounding box
                                    sample_points = []
                                    for dy in range(-h//4, h//4 + 1, h//8):
                                        for dx in range(-w//4, w//4 + 1, w//8):
                                            sample_y = center_y + dy
                                            sample_x = center_x + dx
                                            if (0 <= sample_y < depth_image.shape[0] and 
                                                0 <= sample_x < depth_image.shape[1]):
                                                sample_depth = depth_image[sample_y, sample_x]
                                                if not np.isnan(sample_depth) and sample_depth > 0:
                                                    sample_points.append(sample_depth)
                                    
                                    if sample_points:
                                        depth_value = np.median(sample_points)
                                
                                # If valid depth obtained, calculate 3D coordinates
                                if depth_value is not None and not np.isnan(depth_value) and depth_value > 0:
                                    point_3d = self.pixel_to_3d(center_x, center_y, depth_value)
                                    if point_3d:
                                        distance = np.sqrt(point_3d[0]**2 + point_3d[1]**2 + point_3d[2]**2)
                        
                        # If no depth value, estimate distance
                        if distance is None:
                            # Simple estimation based on area
                            standard_area_at_1m = 10000
                            distance = standard_area_at_1m / area if area > 0 else 0
                        
                        # Format RGB value
                        border_rgb = (bgr_color[2], bgr_color[1], bgr_color[0])  # BGR to RGB
                        
                        # Build detection info
                        detection = {
                            'color_id': color_id,
                            'color_name': color_name,
                            'rgb': border_rgb,
                            'x': int(x),
                            'y': int(y),
                            'width': int(w),
                            'height': int(h),
                            'center_x': int(center_x),
                            'center_y': int(center_y),
                            'frame_center_x': int(frame_cx),
                            'frame_center_y': int(frame_cy),
                            'delta_x': int(frame_cx - center_x),
                            'delta_y': int(frame_cy - center_y),
                            'area': float(area),
                            'aspect_ratio': float(aspect_ratio),
                            'estimated_distance_m': float(distance),
                            'depth_value': float(depth_value) if depth_value is not None else None
                        }
                        
                        # If 3D coordinates exist, add to detection info
                        if point_3d:
                            detection['x_3d'] = float(point_3d[0])
                            detection['y_3d'] = float(point_3d[1])
                            detection['z_3d'] = float(point_3d[2])
                        
                        all_detections.append(detection)
                        
                        # Draw on image
                        self.draw_detection(debug_image, detection, bgr_color)
        
        # Add statistics at top of image
        self.draw_statistics(debug_image, all_detections)
        
        return debug_image, all_detections

    def draw_center_cross(self, image):
        """Draw a small '+' at the center of the frame"""
        height, width = image.shape[:2]
        cx = width // 2
        cy = height // 2
        size = 6
        color = (255, 255, 255)
        thickness = 1
        cv2.line(image, (cx - size, cy), (cx + size, cy), color, thickness, cv2.LINE_AA)
        cv2.line(image, (cx, cy - size), (cx, cy + size), color, thickness, cv2.LINE_AA)
    
    def draw_detection(self, image, detection, bgr_color):
        """Draw detection result on image"""
        x = detection['x']
        y = detection['y']
        w = detection['width']
        h = detection['height']
        center_x = detection['center_x']
        center_y = detection['center_y']
        color_name = detection['color_name']
        distance = detection['estimated_distance_m']
        depth_value = detection.get('depth_value')
        img_h, img_w = image.shape[:2]
        frame_cx = img_w // 2
        frame_cy = img_h // 2
        delta_x = frame_cx - center_x
        delta_y = frame_cy - center_y
        
        # 1. Draw bounding box
        thickness = 3
        cv2.rectangle(image, (x, y), (x + w, y + h), bgr_color, thickness)
        
        # 2. Draw center point
        cv2.circle(image, (center_x, center_y), 5, bgr_color, -1)
        
        # 3. Add label
        font_scale = 0.5
        font_thickness = 1
        line_spacing = 5
        
        # Build label text (all in English)
        label_lines = []
        label_lines.append(f"{color_name}")
        
        if depth_value is not None and not np.isnan(depth_value):
            label_lines.append(f"Depth: {depth_value:.3f}m")
            label_lines.append(f"Distance: {distance:.3f}m")
        else:
            label_lines.append(f"Est. Distance: {distance:.2f}m")
        
        # Calculate text size
        text_sizes = []
        max_width = 0
        total_height = 0
        for line in label_lines:
            (text_width, text_height), _ = cv2.getTextSize(
                line, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness
            )
            text_sizes.append((text_width, text_height))
            max_width = max(max_width, text_width)
            total_height += text_height
        
        # Add line spacing
        total_height += (len(label_lines) - 1) * line_spacing
        
        # Label background position
        bg_y_start = y - total_height - 10
        bg_y_end = y
        bg_x_end = x + max_width + 10
        
        # Ensure background doesn't go above image
        if bg_y_start < 0:
            bg_y_start = 0
            bg_y_end = total_height + 10
        
        # Draw background
        cv2.rectangle(
            image,
            (x, bg_y_start),
            (bg_x_end, bg_y_end),
            bgr_color,
            -1
        )
        
        # Draw each line of text
        current_y = bg_y_start
        for i, line in enumerate(label_lines):
            text_height = text_sizes[i][1]
            current_y += text_height
            
            cv2.putText(
                image,
                line,
                (x + 5, current_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale,
                (255, 255, 255),  # White text
                font_thickness
            )
            
            current_y += line_spacing
        
        # 4. Show center coordinates (coordinates are numbers, no change needed)
        coord_text = f"({center_x}, {center_y})"
        cv2.putText(
            image,
            coord_text,
            (center_x - 25, center_y + 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )

        # 5. Show offset from frame center (pixels)
        offset_text = f"Delta({delta_x}, {delta_y})"
        cv2.putText(
            image,
            offset_text,
            (10, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )
    
    def draw_statistics(self, image, detections):
        """Draw statistics on image"""
        if detections:
            total_detections = len(detections)
            # Group count by color
            color_counts = {}
            for detection in detections:
                color_name = detection['color_name']
                color_counts[color_name] = color_counts.get(color_name, 0) + 1
            
            # Create statistics string
            count_parts = [f"{color}: {count}" for color, count in color_counts.items()]
            stats_detail = ", ".join(count_parts)
            stat_text = f"Objects: {total_detections}"
            
            # First line: total count
            cv2.putText(
                image,
                stat_text,
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),  # White outline
                3
            )
            cv2.putText(
                image,
                stat_text,
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 0),  # Black text
                2
            )
            
            # Second line: detailed statistics
            if stats_detail:
                cv2.putText(
                    image,
                    stats_detail,
                    (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),  # White outline
                    3
                )
                cv2.putText(
                    image,
                    stats_detail,
                    (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (50, 150, 255),  # Orange text
                    2
                )
        else:
            # If no objects detected
            stat_text = "No objects detected"
            cv2.putText(
                image,
                stat_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),  # Red
                2
            )
    
    def publish_detection_info(self, detections, header):
        """Publish detection information"""
        # 1. Publish detected color names
        detected_colors = set([d['color_name'] for d in detections])
        if detected_colors:
            color_names_msg = String()
            color_names_msg.data = ", ".join(detected_colors)
            self.color_names_pub.publish(color_names_msg)
        
        # 2. Publish detailed detection info (JSON format)
        import json
        info_msg = String()
        
        info_dict = {
            'timestamp': self.get_clock().now().to_msg().sec,
            'total_detections': len(detections),
            'detections': detections,
            'summary': {
                color_name: len([d for d in detections if d['color_name'] == color_name])
                for color_name in set([d['color_name'] for d in detections])
            }
        }
        
        info_msg.data = json.dumps(info_dict, ensure_ascii=False)
        self.detection_info_pub.publish(info_msg)
        
        # 3. Publish position information
        position_msg = Float32MultiArray()
        for detection in detections:
            # If 3D coordinates, use 3D; otherwise use 2D
            if 'x_3d' in detection and 'y_3d' in detection:
                position_msg.data.extend([
                    float(detection['x_3d']),
                    float(detection['y_3d']),
                    float(detection.get('z_3d', 0)),
                    float(detection['width']),
                    float(detection['height']),
                    float(1 if detection['color_name'] == 'RED' else 
                          2 if detection['color_name'] == 'GREEN' else 
                          3 if detection['color_name'] == 'BLUE' else 4)
                ])
            else:
                position_msg.data.extend([
                    float(detection['center_x']),
                    float(detection['center_y']),
                    float(0),
                    float(detection['width']),
                    float(detection['height']),
                    float(1 if detection['color_name'] == 'RED' else 
                          2 if detection['color_name'] == 'GREEN' else 
                          3 if detection['color_name'] == 'BLUE' else 4)
                ])
        
        if position_msg.data:
            self.position_pub.publish(position_msg)

        # 3. Publish 3D points (camera frame) for detections with depth.
        header = self._normalize_frame_id(header)
        for detection in detections:
            if 'x_3d' not in detection or 'y_3d' not in detection or 'z_3d' not in detection:
                continue
            pt_msg = PointStamped()
            pt_msg.header = header
            pt_msg.point.x = float(detection['x_3d'])
            pt_msg.point.y = float(detection['y_3d'])
            pt_msg.point.z = float(detection['z_3d'])
            self.point3d_pub.publish(pt_msg)
        
        # 4. Show brief info in terminal (in English)
        self.get_logger().info(
            f"Detected {len(detections)} objects: {', '.join(detected_colors)}",
            throttle_duration_sec=1.0
        )
    
    def __del__(self):
        """Cleanup function"""
        if self.show_debug_window:
            cv2.destroyWindow(self.debug_window_name)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.show_debug_window:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
