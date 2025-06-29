#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Twist
from cv_bridge import CvBridge
import time
import math

class SimpleDataTest(Node):
    def __init__(self):
        super().__init__('simple_data_test')
        self.bridge = CvBridge()
        
        # Latest data
        self.latest_image = None
        self.image_count = 0
        
        # Subscribe to camera only (we know this works)
        self.image_sub = self.create_subscription(Image, '/robosub/camera/simulated_image', self.image_callback, 10)
        
        # Try different gate control topics
        self.gate_pose_pub = self.create_publisher(Pose, '/model/gate/pose', 10)
        self.gate_twist_pub = self.create_publisher(Twist, '/model/gate/twist', 10)
        
        # Timer
        self.timer = self.create_timer(3.0, self.test_gate_and_collect)
        self.position_index = 0
        
        print("🚀 Simple Test: Camera + Gate Movement")
        print("📋 Available data will be printed")
    
    def image_callback(self, msg):
        self.latest_image = msg
        self.image_count += 1
    
    def move_gate_safe(self, x, y, z, yaw=0):
        """Move gate with proper message format"""
        try:
            # Create pose message properly
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)  
            pose.position.z = float(z)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = float(math.sin(yaw/2))
            pose.orientation.w = float(math.cos(yaw/2))
            
            # Stop any movement first
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            
            self.gate_twist_pub.publish(twist)
            self.gate_pose_pub.publish(pose)
            
            print(f"🚪 Commanded gate to: ({x:.1f}, {y:.1f}, {z:.1f}) yaw={yaw:.2f}")
            
        except Exception as e:
            print(f"❌ Gate movement error: {e}")
    
    def test_gate_and_collect(self):
        """Test gate movement and show available data"""
        
        # Show what we have
        if self.latest_image:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
                print(f"📷 Camera: {cv_image.shape} (images received: {self.image_count})")
                
                # THIS IS YOUR TUPLE: (image, gate_pos, robosub_pos)
                # For now: (image, known_gate_pos, None)
                data_tuple = (cv_image, f"gate_pos_{self.position_index}", None)
                print(f"📦 Data tuple ready: image + gate_position + robosub_pose")
                
            except Exception as e:
                print(f"❌ Image conversion error: {e}")
        else:
            print("❌ No camera data yet")
        
        # Move gate to next position  
        positions = [(6.0, 0.0, 1.0), (8.0, 2.0, 2.0), (4.0, -1.0, 1.5), (10.0, 0.0, 3.0)]
        x, y, z = positions[self.position_index % len(positions)]
        self.move_gate_safe(x, y, z, self.position_index * 0.3)
        self.position_index += 1

def main():
    rclpy.init()
    test = SimpleDataTest()
    
    try:
        print("🎮 Testing camera + gate movement...")
        rclpy.spin(test)
    except KeyboardInterrupt:
        print("\n🛑 Test done!")
    finally:
        test.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()