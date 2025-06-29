#!/usr/bin/env python3
"""
Gate Controller - Controls gate position and rotation for robosub simulation
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
import math
import time

class GateController(Node):
    def __init__(self):
        super().__init__('gate_controller')
        
        # Publishers
        self.gate_pose_pub = self.create_publisher(Pose, '/model/gate/pose', 10)
        self.gate_twist_pub = self.create_publisher(Twist, '/model/gate/twist', 10)
        
        # Current gate state
        self.current_position = {'x': 5.0, 'y': 0.0, 'z': 2.0, 'yaw': 0.0}
        
        # Predefined positions for cycling
        self.gate_positions = [
            {'x': 5.0, 'y': 0.0, 'z': 2.0, 'yaw': 0.0},
            {'x': 4.0, 'y': 2.0, 'z': 1.5, 'yaw': 0.5},
            {'x': 6.0, 'y': -1.5, 'z': 2.5, 'yaw': -0.3},
            {'x': 3.5, 'y': 1.0, 'z': 1.0, 'yaw': 0.8},
            {'x': 7.0, 'y': 0.5, 'z': 3.0, 'yaw': -0.5},
        ]
        self.current_index = 0
        
        self.get_logger().info("🚪 Gate Controller Started!")
        self.get_logger().info("📋 Commands available:")
        self.get_logger().info("  - set_position(x, y, z, yaw)")
        self.get_logger().info("  - cycle_position()")
        self.get_logger().info("  - rotate(yaw)")
    
    def set_position(self, x: float, y: float, z: float, yaw: float = 0.0):
        """Set gate to specific position and rotation"""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        
        # Convert yaw to quaternion
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        
        self.gate_pose_pub.publish(pose)
        self.current_position = {'x': x, 'y': y, 'z': z, 'yaw': yaw}
        
        self.get_logger().info(f"🚪 Set gate position: ({x:.2f}, {y:.2f}, {z:.2f}, yaw={yaw:.2f})")
        return self.current_position
    
    def cycle_position(self):
        """Cycle through predefined positions"""
        pos = self.gate_positions[self.current_index]
        self.set_position(pos['x'], pos['y'], pos['z'], pos['yaw'])
        self.current_index = (self.current_index + 1) % len(self.gate_positions)
        return pos
    
    def rotate(self, yaw: float):
        """Rotate gate to specific yaw angle"""
        self.set_position(
            self.current_position['x'],
            self.current_position['y'], 
            self.current_position['z'],
            yaw
        )
    
    def move_relative(self, dx: float, dy: float, dz: float):
        """Move gate relative to current position"""
        new_x = self.current_position['x'] + dx
        new_y = self.current_position['y'] + dy
        new_z = self.current_position['z'] + dz
        self.set_position(new_x, new_y, new_z, self.current_position['yaw'])
    
    def stop_motion(self):
        """Stop any gate movement"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.gate_twist_pub.publish(twist)
        self.get_logger().info("🛑 Gate motion stopped")

def main():
    rclpy.init()
    controller = GateController()
    
    try:
        print("🚪 Gate Controller Interactive Mode")
        print("Available commands:")
        print("  1 - Cycle through positions")
        print("  2 - Set custom position")
        print("  3 - Rotate gate")
        print("  4 - Move relative")
        print("  q - Quit")
        
        # Set initial position
        controller.set_position(5.0, 0.0, 2.0, 0.0)
        
        while rclpy.ok():
            rclpy.spin_once(controller, timeout_sec=0.1)
            
            # Non-blocking input simulation (in real usage, use keyboard input)
            print("\n🎮 Gate Controller Running...")
            print("Commands: 1=cycle, 2=custom, 3=rotate, 4=relative, q=quit")
            
            # Auto-demo: cycle positions every 5 seconds
            time.sleep(5)
            pos = controller.cycle_position()
            print(f"🚪 Auto-cycled to: ({pos['x']}, {pos['y']}, {pos['z']}) yaw={pos['yaw']}")
            
    except KeyboardInterrupt:
        print("\n🛑 Shutting down gate controller...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 