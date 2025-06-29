#!/usr/bin/env python3
"""
Simple Camera Collector - Just collects camera images from robosub simulation
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os
import time
from datetime import datetime

class SimpleCameraCollector(Node):
    def __init__(self):
        super().__init__('simple_camera_collector')
        self.bridge = CvBridge()
        self.image_count = 0
        self.start_time = time.time()
        
        # Create directory in workspace - look for robosub_sim_src
        current_dir = os.getcwd()
        if 'robosub_sim_src' in current_dir:
            # If we're already in robosub_sim_src or subdirectory
            workspace_root = current_dir.split('robosub_sim_src')[0] + 'robosub_sim_src'
        else:
            # Fallback to current directory
            workspace_root = current_dir
            
        self.save_dir = os.path.join(workspace_root, "collected_images")
        os.makedirs(self.save_dir, exist_ok=True)
        
        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image, '/robosub/camera/simulated_image', self.image_callback, 10)
        
        self.get_logger().info("📷 Simple Camera Collector Started!")
        self.get_logger().info("🎯 Collecting from: /robosub/camera/simulated_image")
        self.get_logger().info(f"📁 Saving to: {self.save_dir}/")
    
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Save with timestamp
            timestamp = time.time()
            filename = os.path.join(self.save_dir, f"image_{self.image_count:06d}_{timestamp:.3f}.png")
            cv2.imwrite(filename, cv_image)
            
            self.image_count += 1
            
            # Log progress
            if self.image_count % 10 == 0:
                elapsed = time.time() - self.start_time
                rate = self.image_count / elapsed if elapsed > 0 else 0
                self.get_logger().info(f"📸 Saved {self.image_count} images (Rate: {rate:.1f} Hz)")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error saving image: {e}")

def main():
    rclpy.init()
    collector = SimpleCameraCollector()
    
    try:
        print("🚀 Starting camera collection... Press Ctrl+C to stop")
        print(f"📁 Images will be saved to: {collector.save_dir}")
        rclpy.spin(collector)
    except KeyboardInterrupt:
        elapsed = time.time() - collector.start_time
        rate = collector.image_count / elapsed if elapsed > 0 else 0
        print(f"\n🏁 Collection Complete!")
        print(f"📸 Total images saved: {collector.image_count}")
        print(f"⏱️  Duration: {elapsed:.1f} seconds")
        print(f"📊 Average rate: {rate:.1f} Hz")
        print(f"📁 Images saved in: {collector.save_dir}")
    finally:
        collector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 