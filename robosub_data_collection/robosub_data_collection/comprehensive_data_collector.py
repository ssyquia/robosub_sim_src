#!/usr/bin/env python3
"""
Comprehensive Data Collector for Robosub Simulation

Features:
- Collects camera images
- Controls gate position and rotation
- Makes robosub static
- Publishes pose commands
- Collects synchronized data
- Returns poses of both robosub and gate
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os
import json
import time
import math
from typing import Dict, List, Tuple, Optional
import threading


class ComprehensiveDataCollector(Node):
    """
    Complete data collection system that:
    1. Controls gate position/rotation
    2. Makes robosub static
    3. Collects camera + pose data
    4. Provides comprehensive data output
    """
    
    def __init__(self):
        super().__init__('comprehensive_data_collector')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Data storage
        self.collected_data: List[Dict] = []
        self.latest_robosub_pose: Optional[Pose] = None
        self.latest_gate_pose: Optional[Pose] = None
        self.latest_image: Optional[Image] = None
        
        # Configuration
        self.data_dir = "comprehensive_data"
        self.images_dir = os.path.join(self.data_dir, "images")
        self.metadata_file = os.path.join(self.data_dir, "metadata.json")
        
        # Create directories
        os.makedirs(self.images_dir, exist_ok=True)
        
        # Control state
        self.robosub_static = True
        self.collecting_data = False
        self.image_count = 0
        self.start_time = time.time()
        
        # Gate control
        self.current_gate_position = {'x': 5.0, 'y': 0.0, 'z': 2.0, 'yaw': 0.0}
        self.gate_positions = [
            {'x': 5.0, 'y': 0.0, 'z': 2.0, 'yaw': 0.0},
            {'x': 4.0, 'y': 2.0, 'z': 1.5, 'yaw': 0.5},
            {'x': 6.0, 'y': -1.5, 'z': 2.5, 'yaw': -0.3},
            {'x': 3.5, 'y': 1.0, 'z': 1.0, 'yaw': 0.8},
            {'x': 7.0, 'y': 0.5, 'z': 3.0, 'yaw': -0.5},
        ]
        self.current_gate_index = 0
        
        # Thread safety
        self.data_lock = threading.Lock()
        
        # === SUBSCRIBERS ===
        self.image_sub = self.create_subscription(
            Image, '/robosub/camera/simulated_image', self.image_callback, 10)
        
        # Try multiple pose topic possibilities
        self.robosub_pose_sub = self.create_subscription(
            Pose, '/model/high_level_robosub/pose', self.robosub_pose_callback, 10)
        
        self.gate_pose_sub = self.create_subscription(
            Pose, '/model/gate/pose', self.gate_pose_callback, 10)
        
        # === PUBLISHERS ===
        # Gate control publishers
        self.gate_pose_pub = self.create_publisher(Pose, '/model/gate/pose', 10)
        self.gate_twist_pub = self.create_publisher(Twist, '/model/gate/twist', 10)
        
        # Robosub control publishers (to make static)
        self.robosub_pose_pub = self.create_publisher(Pose, '/model/high_level_robosub/pose', 10)
        self.robosub_twist_pub = self.create_publisher(Twist, '/model/high_level_robosub/twist', 10)
        
        # Control status publisher
        self.status_pub = self.create_publisher(Bool, '/data_collector/collecting', 10)
        
        # === TIMERS ===
        self.control_timer = self.create_timer(0.5, self.publish_control_commands)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.data_timer = self.create_timer(0.1, self.collect_data_point)
        
        self.get_logger().info("🚀 Comprehensive Data Collector Started!")
        self.get_logger().info(f"📁 Data directory: {self.data_dir}")
        self.get_logger().info(f"🎯 Camera topic: /robosub/camera/simulated_image")
        self.get_logger().info(f"🤖 Robosub static mode: {self.robosub_static}")
        
    def image_callback(self, msg: Image):
        """Handle incoming camera images"""
        try:
            with self.data_lock:
                self.latest_image = msg
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")
    
    def robosub_pose_callback(self, msg: Pose):
        """Handle robosub pose messages"""
        try:
            with self.data_lock:
                self.latest_robosub_pose = msg
        except Exception as e:
            self.get_logger().error(f"Error in robosub pose callback: {e}")
    
    def gate_pose_callback(self, msg: Pose):
        """Handle gate pose messages"""
        try:
            with self.data_lock:
                self.latest_gate_pose = msg
        except Exception as e:
            self.get_logger().error(f"Error in gate pose callback: {e}")
    
    def set_gate_position(self, x: float, y: float, z: float, yaw: float = 0.0):
        """Set gate position and rotation"""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        
        # Convert yaw to quaternion
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        
        self.gate_pose_pub.publish(pose)
        self.current_gate_position = {'x': x, 'y': y, 'z': z, 'yaw': yaw}
        
        self.get_logger().info(f"🚪 Set gate position: ({x:.2f}, {y:.2f}, {z:.2f}, yaw={yaw:.2f})")
    
    def cycle_gate_position(self):
        """Cycle through predefined gate positions"""
        gate_pos = self.gate_positions[self.current_gate_index]
        self.set_gate_position(
            gate_pos['x'], gate_pos['y'], gate_pos['z'], gate_pos['yaw']
        )
        self.current_gate_index = (self.current_gate_index + 1) % len(self.gate_positions)
        return gate_pos
    
    def set_robosub_static(self, static: bool = True):
        """Make robosub static or allow movement"""
        self.robosub_static = static
        
        if static:
            # Publish zero twist to stop movement
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.robosub_twist_pub.publish(twist)
        
        self.get_logger().info(f"🤖 Robosub static mode: {static}")
    
    def get_both_poses(self) -> Tuple[Optional[Pose], Optional[Pose]]:
        """Get current poses of both robosub and gate"""
        with self.data_lock:
            return self.latest_robosub_pose, self.latest_gate_pose
    
    def get_camera_output(self) -> Optional[Tuple[Dict, any]]:
        """Get latest camera image as OpenCV array with metadata"""
        if self.latest_image is None:
            return None
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
            metadata = {
                'timestamp': time.time(),
                'height': self.latest_image.height,
                'width': self.latest_image.width,
                'encoding': self.latest_image.encoding,
                'frame_id': self.latest_image.header.frame_id
            }
            return metadata, cv_image
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return None
    
    def capture_synchronized_data(self) -> Dict:
        """Capture synchronized camera image and both poses"""
        timestamp = time.time()
        
        # Get image data
        camera_data = self.get_camera_output()
        
        # Get pose data
        robosub_pose, gate_pose = self.get_both_poses()
        
        data_point = {
            'timestamp': timestamp,
            'camera_available': camera_data is not None,
            'robosub_pose_available': robosub_pose is not None,
            'gate_pose_available': gate_pose is not None,
            'gate_position_commanded': self.current_gate_position.copy()
        }
        
        if camera_data:
            metadata, cv_image = camera_data
            data_point['camera_metadata'] = metadata
            data_point['image_shape'] = cv_image.shape
        
        if robosub_pose:
            data_point['robosub_pose'] = {
                'position': {
                    'x': robosub_pose.position.x,
                    'y': robosub_pose.position.y,
                    'z': robosub_pose.position.z
                },
                'orientation': {
                    'x': robosub_pose.orientation.x,
                    'y': robosub_pose.orientation.y,
                    'z': robosub_pose.orientation.z,
                    'w': robosub_pose.orientation.w
                }
            }
        
        if gate_pose:
            data_point['gate_pose'] = {
                'position': {
                    'x': gate_pose.position.x,
                    'y': gate_pose.position.y,
                    'z': gate_pose.position.z
                },
                'orientation': {
                    'x': gate_pose.orientation.x,
                    'y': gate_pose.orientation.y,
                    'z': gate_pose.orientation.z,
                    'w': gate_pose.orientation.w
                }
            }
        
        return data_point
    
    def collect_data_point(self):
        """Collect a single data point if collection is active"""
        if not self.collecting_data:
            return
        
        try:
            data_point = self.capture_synchronized_data()
            
            # Save image if available
            if data_point['camera_available']:
                camera_data = self.get_camera_output()
                if camera_data:
                    metadata, cv_image = camera_data
                    
                    # Save image
                    timestamp_str = f"{data_point['timestamp']:.6f}"
                    image_filename = f"image_{self.image_count:06d}_{timestamp_str}.png"
                    image_path = os.path.join(self.images_dir, image_filename)
                    
                    cv2.imwrite(image_path, cv_image)
                    
                    # Add image info to data point
                    data_point['image_filename'] = image_filename
                    data_point['image_path'] = image_path
                    
                    self.image_count += 1
            
            # Store data point
            self.collected_data.append(data_point)
            
            # Save metadata periodically
            if len(self.collected_data) % 10 == 0:
                self.save_metadata()
            
            # Log progress
            if len(self.collected_data) % 25 == 0:
                self.log_progress()
                
        except Exception as e:
            self.get_logger().error(f"Error collecting data point: {e}")
    
    def log_progress(self):
        """Log collection progress"""
        elapsed = time.time() - self.start_time
        rate = len(self.collected_data) / elapsed if elapsed > 0 else 0
        
        # Count successful data types
        with_camera = sum(1 for d in self.collected_data if d['camera_available'])
        with_robosub = sum(1 for d in self.collected_data if d['robosub_pose_available'])
        with_gate = sum(1 for d in self.collected_data if d['gate_pose_available'])
        
        self.get_logger().info(
            f"📊 Collected {len(self.collected_data)} points "
            f"(Rate: {rate:.1f} Hz) - "
            f"Camera: {with_camera}, Robosub: {with_robosub}, Gate: {with_gate}"
        )
    
    def start_data_collection(self):
        """Start collecting data"""
        self.collecting_data = True
        self.start_time = time.time()
        self.get_logger().info("🎬 Started data collection")
    
    def stop_data_collection(self):
        """Stop collecting data and save final metadata"""
        self.collecting_data = False
        self.save_metadata()
        self.log_progress()
        self.get_logger().info("🛑 Stopped data collection")
    
    def save_metadata(self):
        """Save metadata to JSON file"""
        try:
            metadata = {
                "collection_info": {
                    "start_time": self.start_time,
                    "total_data_points": len(self.collected_data),
                    "images_saved": self.image_count,
                    "robosub_static": self.robosub_static,
                    "gate_positions_used": self.gate_positions
                },
                "data": self.collected_data
            }
            
            with open(self.metadata_file, 'w') as f:
                json.dump(metadata, f, indent=2)
                
        except Exception as e:
            self.get_logger().error(f"Error saving metadata: {e}")
    
    def publish_control_commands(self):
        """Publish control commands to maintain system state"""
        if self.robosub_static:
            self.set_robosub_static(True)
    
    def publish_status(self):
        """Publish collection status"""
        status_msg = Bool()
        status_msg.data = self.collecting_data
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    collector = ComprehensiveDataCollector()
    
    try:
        print("🚀 Comprehensive Data Collector Started!")
        print("📋 Available commands:")
        print("  - Data collection will start automatically")
        print("  - Gate positions will cycle every 10 seconds")
        print("  - Press Ctrl+C to stop and save data")
        print()
        
        # Set initial state
        collector.set_robosub_static(True)
        time.sleep(1)  # Let it settle
        
        # Start data collection
        collector.start_data_collection()
        
        # Main loop with gate position cycling
        gate_timer = 0
        while rclpy.ok():
            rclpy.spin_once(collector, timeout_sec=0.1)
            
            gate_timer += 0.1
            if gate_timer >= 10.0:  # Change gate position every 10 seconds
                new_pos = collector.cycle_gate_position()
                print(f"🚪 Moved gate to: ({new_pos['x']}, {new_pos['y']}, {new_pos['z']}) yaw={new_pos['yaw']}")
                gate_timer = 0
                
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
    finally:
        collector.stop_data_collection()
        collector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 