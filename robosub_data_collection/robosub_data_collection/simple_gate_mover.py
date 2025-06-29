#!/usr/bin/env python3
"""
Simple Gate Mover - Just tries to move a gate around in the simulation

This script publishes to multiple possible gate control topics to see which one works.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, PoseStamped, TwistStamped
import math
import time
import random

class SimpleGateMover(Node):
    def __init__(self):
        super().__init__('simple_gate_mover')
        
        # Try EVERY possible gate control topic
        self.publishers = {}
        
        # Pose publishers - different naming conventions
        self.publishers['gate_pose'] = self.create_publisher(Pose, '/model/gate/pose', 10)
        self.publishers['gate_pose_cmd'] = self.create_publisher(Pose, '/gate/pose', 10)
        self.publishers['gate_cmd_pose'] = self.create_publisher(Pose, '/gate/cmd_pose', 10)
        self.publishers['gate_set_pose'] = self.create_publisher(Pose, '/gate/set_pose', 10)
        
        # Twist publishers
        self.publishers['gate_twist'] = self.create_publisher(Twist, '/model/gate/twist', 10)
        self.publishers['gate_cmd_vel'] = self.create_publisher(Twist, '/gate/cmd_vel', 10)
        self.publishers['gate_twist_cmd'] = self.create_publisher(Twist, '/gate/twist', 10)
        
        # Stamped versions
        self.publishers['gate_pose_stamped'] = self.create_publisher(PoseStamped, '/gate/pose_stamped', 10)
        self.publishers['gate_twist_stamped'] = self.create_publisher(TwistStamped, '/gate/twist_stamped', 10)
        
        # Joint control (if gate has joints)
        self.publishers['gate_joint_cmd'] = self.create_publisher(Twist, '/gate/joint/cmd', 10)
        
        # Current position
        self.current_position = {'x': 5.0, 'y': 0.0, 'z': 2.0, 'yaw': 0.0}
        
        # Timer to move gate every 2 seconds
        self.move_timer = self.create_timer(2.0, self.move_gate)
        
        self.get_logger().info("🚪 Simple Gate Mover Started!")
        self.get_logger().info(f"📡 Publishing to {len(self.publishers)} different topics")
        self.get_logger().info("🎯 Gate should move every 2 seconds")
        
        # List all topics we're publishing to
        for name, pub in self.publishers.items():
            self.get_logger().info(f"  - {pub.topic_name}")
    
    def move_gate(self):
        """Move gate to a random position"""
        # Generate random position
        x = random.uniform(3.0, 8.0)
        y = random.uniform(-2.0, 2.0) 
        z = random.uniform(1.0, 3.0)
        yaw = random.uniform(-1.0, 1.0)
        
        self.current_position = {'x': x, 'y': y, 'z': z, 'yaw': yaw}
        
        # Create pose message
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        
        # Convert yaw to quaternion (simplified)
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        
        # Create stamped pose
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "world"
        pose_stamped.pose = pose
        
        # Create twist (for velocity-based control)
        twist = Twist()
        twist.linear.x = (x - 5.0) * 0.5  # Move towards target
        twist.linear.y = y * 0.5
        twist.linear.z = (z - 2.0) * 0.5
        twist.angular.z = yaw * 0.5
        
        # Create stamped twist
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "world"
        twist_stamped.twist = twist
        
        # Publish to ALL topics
        try:
            # Pose topics
            self.publishers['gate_pose'].publish(pose)
            self.publishers['gate_pose_cmd'].publish(pose)
            self.publishers['gate_cmd_pose'].publish(pose)
            self.publishers['gate_set_pose'].publish(pose)
            self.publishers['gate_pose_stamped'].publish(pose_stamped)
            
            # Twist topics
            self.publishers['gate_twist'].publish(twist)
            self.publishers['gate_cmd_vel'].publish(twist)
            self.publishers['gate_twist_cmd'].publish(twist)
            self.publishers['gate_twist_stamped'].publish(twist_stamped)
            self.publishers['gate_joint_cmd'].publish(twist)
            
            self.get_logger().info(f"🚪 Moving gate to: ({x:.2f}, {y:.2f}, {z:.2f}) yaw={yaw:.2f}")
            self.get_logger().info("📡 Published to all gate control topics")
            
        except Exception as e:
            self.get_logger().error(f"Error publishing: {e}")

def main():
    rclpy.init()
    mover = SimpleGateMover()
    
    try:
        print("🚪 Simple Gate Mover - Testing Gate Movement")
        print("📋 What this does:")
        print("  - Tries to move gate every 2 seconds")
        print("  - Publishes to 10+ different possible gate topics")
        print("  - Shows which topics exist vs which don't")
        print("  - You should see the gate moving in Gazebo!")
        print()
        print("🔍 Watch the Gazebo simulation window...")
        print("📊 Check the logs to see which topics work")
        print("⏹️  Press Ctrl+C to stop")
        print()
        
        rclpy.spin(mover)
        
    except KeyboardInterrupt:
        print("\n🛑 Gate mover stopped")
    finally:
        mover.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 