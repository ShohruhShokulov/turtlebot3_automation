#!/bin/bash
# Smart Campus Delivery - Object Detection Test
# Simulates YOLO-based detection of campus obstacles
# Detects: person, bicycle, box (packages) for safe navigation

echo "=========================================="
echo "Campus Delivery - Obstacle Detection Test"
echo "=========================================="
echo "Testing: Real-time obstacle detection"
echo "Detecting: person, bicycle, box (packages)"
echo "Purpose: Safe navigation during campus deliveries"
echo ""
echo "This script simulates camera feed and YOLO detections"
echo "Press Ctrl+C to stop"
echo "=========================================="
echo ""

# Create a temporary Python script to publish test images and simulate detections
python3 << 'EOF'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import sys
import random
import time

class CampusObstacleSimulator(Node):
    def __init__(self):
        super().__init__('campus_obstacle_simulator')
        self.image_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.detection_pub = self.create_publisher(String, 'campus_delivery/obstacle_alerts', 10)
        self.timer = self.create_timer(3.0, self.simulate_detection)
        
        self.get_logger().info('='*70)
        self.get_logger().info('Campus Obstacle Detection Simulator')
        self.get_logger().info('Publishing to: /camera/color/image_raw')
        self.get_logger().info('Detection alerts: campus_delivery/obstacle_alerts')
        self.get_logger().info('='*70)
        self.get_logger().info('')
        
        # Campus-specific obstacles
        self.campus_obstacles = [
            ('person', 'Student crossing pathway'),
            ('person', 'Faculty member detected'),
            ('bicycle', 'Bicycle blocking route'),
            ('bicycle', 'Parked bike detected'),
            ('backpack', 'Large backpack on pathway'),
            ('suitcase', 'Package/luggage detected'),
            ('person', 'Group of students ahead'),
        ]
        self.detection_count = 0
        
    def simulate_detection(self):
        """Simulate random campus obstacle detection"""
        # Publish synthetic camera image
        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_color_optical_frame'
        img_msg.height = 480
        img_msg.width = 640
        img_msg.encoding = 'bgr8'
        img_msg.is_bigendian = 0
        img_msg.step = 640 * 3
        
        # Create synthetic image
        img = np.ones((480, 640, 3), dtype=np.uint8) * 128
        img_msg.data = img.tobytes()
        self.image_pub.publish(img_msg)
        
        # Simulate detection
        if random.random() > 0.3:  # 70% chance of detecting something
            obstacle, description = random.choice(self.campus_obstacles)
            confidence = round(random.uniform(0.65, 0.95), 2)
            
            self.detection_count += 1
            
            # Determine icon
            if obstacle == 'person':
                icon = '🚶'
            elif obstacle == 'bicycle':
                icon = '🚲'
            else:
                icon = '📦'
            
            detection_msg = f"{obstacle}:{confidence}"
            
            self.get_logger().info(
                f'{icon} OBSTACLE DETECTED #{self.detection_count}: {obstacle.upper()} '
                f'(confidence: {confidence:.2f})'
            )
            self.get_logger().info(f'   └─ {description}')
            
            # Publish detection alert
            alert_msg = String()
            alert_msg.data = detection_msg
            self.detection_pub.publish(alert_msg)
            
            # Safety recommendation
            if obstacle == 'person':
                self.get_logger().info('   └─ ACTION: Slow down, give right of way')
            elif obstacle == 'bicycle':
                self.get_logger().info('   └─ ACTION: Navigate around obstacle')
            else:
                self.get_logger().info('   └─ ACTION: Avoid collision with package')
                
            self.get_logger().info('')
        else:
            self.get_logger().info('✓ Path clear - No obstacles detected')
            self.get_logger().info('')

def main():
    rclpy.init()
    node = CampusObstacleSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('')
        node.get_logger().info('='*70)
        node.get_logger().info(f'Campus Obstacle Simulator stopped')
        node.get_logger().info(f'Total obstacles detected: {node.detection_count}')
        node.get_logger().info('='*70)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
