#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Point
from ultralytics import YOLO

class HumanDetectorWithPointCloud(Node):
    def __init__(self):
        super().__init__('yolo_human_detector')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Declare parameters
        self.declare_parameter('rgb_topic', '/orbbec_astra/front_rgbd_camerargb/image_raw_color')
        self.declare_parameter('pointcloud_topic', '/orbbec_astra/front_rgbd_camerapoint_cloud/cloud_registered')
        self.declare_parameter('model_path', 'yolov8n.pt')
        
        # Get parameters
        self.rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.confidence_threshold = 0.5
        self.iou_threshold = 0.4
        
        # Store latest point cloud for distance calculation
        self.latest_pointcloud = None
        
        # Load YOLO model
        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info(f"YOLO model {self.model_path} loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            return
        
        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.rgb_topic,
            self.image_callback,
            10
        )
        
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self.pointcloud_callback,
            10
        )
        
        # Create publishers
        self.detection_pub = self.create_publisher(String, '/human_detections_status', 10)
        self.human_distance_pub = self.create_publisher(Float64, '/human_distance', 10)
        
        self.get_logger().info(f"set up complete")
    
    def pointcloud_callback(self, msg):
        self.latest_pointcloud = msg
    
    def image_callback(self, msg):
        """Process RGB image for human detection"""
        try:
            # Convert ROS image to OpenCV format
            cv_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Run YOLO detection on RGB image
            results = self.model(cv_rgb, conf=self.confidence_threshold, iou=self.iou_threshold)
            
            # Process detections
            human_detections = []
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    # Filter for person class (class 0 in COCO dataset)
                    human_mask = boxes.cls == 0
                    if human_mask.any():
                        human_boxes = boxes[human_mask]
                        human_detections = human_boxes
                        break
            
            if len(human_detections) > 0:
                self.process_human_detections_with_pointcloud(cv_rgb, human_detections, msg.header)
            else:
                detection_msg = String()
                detection_msg.data = "No humans detected"
                self.detection_pub.publish(detection_msg)
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    
    def get_distance_from_pointcloud(self, x, y, radius=10):
        if self.latest_pointcloud is None:
            return None
        
        try:
            # Get point cloud dimensions
            height = self.latest_pointcloud.height
            width = self.latest_pointcloud.width
            
            # Check if point cloud is organized
            if height == 1:
                # Unorganized point cloud, cannot use pixel coordinates
                self.get_logger().warn("Point cloud is unorganized, cannot extract distance at pixel coordinates")
                return None
            
            # Ensure coordinates are within bounds
            x = max(radius, min(width - radius - 1, x))
            y = max(radius, min(height - radius - 1, y))
            
            # Extract points in a region around the target pixel
            distances = []
            
            # Read all points from the point cloud first
            try:
                all_points = list(pc2.read_points(
                    self.latest_pointcloud,
                    field_names=("x", "y", "z"),
                    skip_nans=True
                ))
                
                # Convert to organized array if possible
                if len(all_points) == height * width:
                    # Organized point cloud
                    for dy in range(-radius, radius + 1):
                        for dx in range(-radius, radius + 1):
                            pixel_x = x + dx
                            pixel_y = y + dy
                            
                            # Check bounds
                            if 0 <= pixel_x < width and 0 <= pixel_y < height:
                                # Calculate index in the point cloud
                                idx = pixel_y * width + pixel_x
                                
                                if idx < len(all_points):
                                    point = all_points[idx]
                                    # Check if point is valid (not NaN)
                                    if not (np.isnan(point[0]) or np.isnan(point[1]) or np.isnan(point[2])):
                                        # Calculate Euclidean distance from camera origin
                                        distance = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
                                        if 0.1 < distance < 10.0:  # Valid distance range
                                            distances.append(distance)
                else:
                    # Unorganized point cloud, try to find closest points
                    center_point = None
                    if width * height > 0:
                        center_idx = y * width + x
                        if center_idx < len(all_points):
                            center_point = all_points[center_idx]
                    
                    if center_point and not any(np.isnan(center_point)):
                        distance = np.sqrt(center_point[0]**2 + center_point[1]**2 + center_point[2]**2)
                        if 0.1 < distance < 10.0:
                            return distance
                
            except Exception as e:
                self.get_logger().error(f"Error reading point cloud: {e}")
                return None
            
            if distances:
                return float(np.median(distances))
            else:
                return None
                
        except Exception as e:
            self.get_logger().error(f"Error extracting distance from point cloud: {e}")
            return None
    
    def process_human_detections_with_pointcloud(self, rgb_image, detections, header):
        detection_info = []
        annotated_image = rgb_image.copy()
        
        for i in range(len(detections)):
            # Extract bounding box coordinates (xyxy format)
            box = detections.xyxy[i].cpu().numpy()
            confidence = detections.conf[i].cpu().numpy()
            
            x1, y1, x2, y2 = map(int, box)
            
            # Calculate center point
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            
            # Get distance from point cloud at center point
            distance = self.get_distance_from_pointcloud(center_x, center_y)
                        
            if distance is not None:
                if i == 0:
                    distance_msg = Float64()
                    distance_msg.data = distance
                    self.human_distance_pub.publish(distance_msg)
                        
            # Store detection info
            detection_info.append({
                'bbox': [x1, y1, x2, y2],
                'center': [center_x, center_y],
                'confidence': float(confidence),
                'distance': distance
            })
                    
        # Publish detection results
        detection_msg = String()
        detection_msg.data = f"Detected {len(detection_info)} human(s): {detection_info}"
        self.detection_pub.publish(detection_msg)
                
        # when humans are detected
        if detection_info:
            distances = [d['distance'] for d in detection_info if d['distance'] is not None]
            if distances:
                self.get_logger().info(f"Detected {len(detection_info)} person(s) at distances: {[f'{d:.2f}m' for d in distances]}")
            else:
                self.get_logger().info(f"Detected {len(detection_info)} person(s) - no depth data")

def main(args=None):
    rclpy.init(args=args)
    
    detector = HumanDetectorWithPointCloud()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()