#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import math
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from std_msgs.msg import String, Float64
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import TransformListener, Buffer

class PersonFollowingNavigator(Node):
    def __init__(self):
        super().__init__('person_following_navigator')
        
        # Declare parameters
        self.declare_parameter('person_radius', 2.5)  # Distance to maintain from person
        self.declare_parameter('base_frame', 'summit_xl_base_footprint')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_frame', 'summit_xl_front_rgbd_camera_link')
        self.declare_parameter('navigation_timeout', 30.0)  # Navigation timeout in seconds
        self.declare_parameter('min_person_distance', 0.5)  # Minimum distance to consider valid
        self.declare_parameter('max_person_distance', 10.0)  # Maximum distance to consider valid
        
        # Get parameters
        self.person_radius = self.get_parameter('person_radius').get_parameter_value().double_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.navigation_timeout = self.get_parameter('navigation_timeout').get_parameter_value().double_value
        self.min_person_distance = self.get_parameter('min_person_distance').get_parameter_value().double_value
        self.max_person_distance = self.get_parameter('max_person_distance').get_parameter_value().double_value
        
        # Initialize Nav2
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.person_center_sub = self.create_subscription(
            Point,
            '/person_center',
            self.person_center_callback,
            10
        )
        
        self.person_distance_sub = self.create_subscription(
            Float64,
            '/person_distance',
            self.person_distance_callback,
            10
        )
        
        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/person_goal', 10)
        
        # State variables
        self.last_person_center = None
        self.last_person_distance = None
        self.navigation_active = False
        self.last_navigation_time = self.get_clock().now()
        
        # Timer for periodic checks
        self.timer = self.create_timer(1.0, self.navigation_timer_callback)
        
        self.get_logger().info("Person Following Navigator initialized")
        self.get_logger().info(f"Target distance from person: {self.person_radius}m")
    
    def person_center_callback(self, msg):
        """Callback for person center point (pixel coordinates + distance)"""
        self.last_person_center = msg
        
        # Check if we have both center and distance data
        if self.last_person_distance is not None:
            self.process_person_navigation()
    
    def person_distance_callback(self, msg):
        """Callback for person distance"""
        self.last_person_distance = msg.data
        
        # Check if we have both center and distance data
        if self.last_person_center is not None:
            self.process_person_navigation()
    
    def pixel_to_camera_frame(self, pixel_x, pixel_y, distance):
        """Convert pixel coordinates and distance to 3D point in camera frame"""
        # Note: This is a simplified conversion. For accurate results, you'd need
        # camera intrinsics. Assuming camera is pointing forward.
        
        # Simplified conversion (assumes camera center and no distortion)
        # You might need to adjust these based on your camera setup
        camera_angle_h = 60.0  # Horizontal field of view in degrees
        camera_angle_v = 45.0  # Vertical field of view in degrees
        
        # Assuming image center at (320, 240) for typical VGA
        image_center_x = 320
        image_center_y = 240
        
        # Calculate angles from image center
        dx = pixel_x - image_center_x
        dy = pixel_y - image_center_y
        
        # Convert to radians
        angle_h = math.radians(camera_angle_h) * dx / image_center_x
        angle_v = math.radians(camera_angle_v) * dy / image_center_y
        
        # Calculate 3D coordinates in camera frame
        x = distance * math.cos(angle_v) * math.sin(angle_h)
        y = -distance * math.cos(angle_v) * math.cos(angle_h)  # Forward is -Y in camera frame
        z = distance * math.sin(angle_v)
        
        return x, y, z
    
    def create_navigation_goal(self, person_x, person_y, person_z):
        """Create a navigation goal at specified distance from person"""
        try:
            # Create a point in camera frame
            person_point = PoseStamped()
            person_point.header.frame_id = self.camera_frame
            person_point.header.stamp = self.get_clock().now().to_msg()
            person_point.pose.position.x = person_x
            person_point.pose.position.y = person_y
            person_point.pose.position.z = person_z
            person_point.pose.orientation.w = 1.0
            
            # Transform person position to map frame
            try:
                person_in_map = self.tf_buffer.transform(person_point, self.map_frame, Duration(seconds=1.0))
            except Exception as e:
                self.get_logger().error(f"Failed to transform person position to map frame: {e}")
                return None
            
            # Get robot position in map frame
            try:
                robot_transform = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, rclpy.time.Time())
                robot_x = robot_transform.transform.translation.x
                robot_y = robot_transform.transform.translation.y
            except Exception as e:
                self.get_logger().error(f"Failed to get robot position: {e}")
                return None
            
            # Calculate direction from robot to person
            person_map_x = person_in_map.pose.position.x
            person_map_y = person_in_map.pose.position.y
            
            direction_x = person_map_x - robot_x
            direction_y = person_map_y - robot_y
            
            # Normalize direction
            direction_length = math.sqrt(direction_x**2 + direction_y**2)
            if direction_length == 0:
                return None
            
            direction_x /= direction_length
            direction_y /= direction_length
            
            # Calculate goal position (person_radius meters before the person)
            goal_x = person_map_x - direction_x * self.person_radius
            goal_y = person_map_y - direction_y * self.person_radius
            
            # Create goal pose
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = self.map_frame
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = goal_x
            goal_pose.pose.position.y = goal_y
            goal_pose.pose.position.z = 0.0
            
            # Set orientation to face the person
            goal_angle = math.atan2(direction_y, direction_x)
            goal_pose.pose.orientation.z = math.sin(goal_angle / 2.0)
            goal_pose.pose.orientation.w = math.cos(goal_angle / 2.0)
            
            return goal_pose
            
        except Exception as e:
            self.get_logger().error(f"Error creating navigation goal: {e}")
            return None
    
    def process_person_navigation(self):
        """Process person detection and create navigation goal"""
        if self.last_person_center is None or self.last_person_distance is None:
            return
        
        # Validate distance
        if (self.last_person_distance < self.min_person_distance or 
            self.last_person_distance > self.max_person_distance):
            return
        
        # Check if we're already close enough
        if abs(self.last_person_distance - self.person_radius) < 0.5:
            self.get_logger().info(f"Already at target distance ({self.last_person_distance:.2f}m)")
            return
        
        # Convert pixel coordinates to camera frame
        pixel_x = self.last_person_center.x
        pixel_y = self.last_person_center.y
        
        # Convert to 3D point in camera frame
        person_x, person_y, person_z = self.pixel_to_camera_frame(
            pixel_x, pixel_y, self.last_person_distance)
        
        # Create navigation goal
        goal_pose = self.create_navigation_goal(person_x, person_y, person_z)
        
        if goal_pose is not None:
            # Publish goal for visualization
            self.goal_pub.publish(goal_pose)
            
            # Send goal to Nav2 if not already navigating
            if not self.navigation_active:
                self.navigator.goToPose(goal_pose)
                self.navigation_active = True
                self.last_navigation_time = self.get_clock().now()
                self.get_logger().info(f"Navigating to person at distance {self.last_person_distance:.2f}m")
        
        # Reset data to avoid reprocessing
        self.last_person_center = None
        self.last_person_distance = None
    
    def navigation_timer_callback(self):
        """Periodic timer to check navigation status"""
        if self.navigation_active:
            # Check if navigation is complete
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info("Navigation to person completed successfully")
                elif result == TaskResult.CANCELED:
                    self.get_logger().warn("Navigation was canceled")
                elif result == TaskResult.FAILED:
                    self.get_logger().error("Navigation failed")
                
                self.navigation_active = False
            
            # Check for timeout
            elif (self.get_clock().now() - self.last_navigation_time).nanoseconds / 1e9 > self.navigation_timeout:
                self.get_logger().warn("Navigation timeout, canceling current goal")
                self.navigator.cancelTask()
                self.navigation_active = False

def main(args=None):
    rclpy.init(args=args)
    
    navigator = PersonFollowingNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()