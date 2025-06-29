#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
import math
import time

class GateTestMover(Node):
    def __init__(self):
        super().__init__('gate_test_mover')
        
        # Publishers for different possible topics
        self.pose_pub = self.create_publisher(Pose, '/model/gate/pose', 10)
        self.twist_pub = self.create_publisher(Twist, '/model/gate/twist', 10)
        
        # Try alternative topic names too
        self.cmd_pos_pub = self.create_publisher(Pose, '/model/gate/cmd_pos', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/model/gate/cmd_vel', 10)
        
        self.get_logger().info("🚪 Gate Test Mover Started!")
        
    def move_gate_to(self, x, y, z, yaw=0.0):
        """Move gate to specific position"""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        
        # Convert yaw to quaternion
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        
        # Try publishing to different topics
        self.pose_pub.publish(pose)
        self.cmd_pos_pub.publish(pose)
        
        print(f"🎯 Commanded gate to: ({x:.1f}, {y:.1f}, {z:.1f}) yaw={yaw:.2f}")
    
    def stop_gate(self):
        """Stop gate movement"""
        twist = Twist()
        self.twist_pub.publish(twist)
        self.cmd_vel_pub.publish(twist)
        print("🛑 Stopped gate movement")

def main():
    rclpy.init()
    mover = GateTestMover()
    
    print("🚪 Gate Movement Test!")
    print("🎮 Moving gate through different positions...")
    
    try:
        positions = [
            (6.0, -2.0, 0.9, 0.0),    # Original position
            (8.0, 0.0, 2.0, 0.5),     # Move right and up
            (4.0, 2.0, 1.5, -0.3),    # Move left and forward
            (10.0, -3.0, 3.0, 1.0),   # Far right
            (6.0, -2.0, 0.9, 0.0),    # Back to start
        ]
        
        for i, (x, y, z, yaw) in enumerate(positions):
            print(f"\n📍 Position {i+1}/5:")
            mover.move_gate_to(x, y, z, yaw)
            
            # Spin for a bit to send the message
            for _ in range(50):
                rclpy.spin_once(mover, timeout_sec=0.1)
            
            print("⏱️  Waiting 3 seconds...")
            time.sleep(3)
        
        print("\n✅ Test complete! Gate should have moved around.")
        
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted!")
    finally:
        mover.stop_gate()
        mover.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()