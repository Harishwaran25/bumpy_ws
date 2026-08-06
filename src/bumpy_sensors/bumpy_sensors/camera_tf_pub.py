#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class CameraTfPublisher(Node):
    def __init__(self):
        super().__init__('camera_tf_publisher')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer to publish transform at 10Hz
        self.timer = self.create_timer(0.1, self.publish_transform)
        
        self.get_logger().info("Camera TF publisher started")
        
        # Camera position relative to camera_link
        # Adjust these values based on your robot's URDF
        self.camera_position = {
            'x': 0.0,      # meters forward from camera_link
            'y': 0.0,      # meters left from camera_link
            'z': 0.0,      # meters up from camera_link
            'roll': 0.0,   # radians
            'pitch': 0.0,  # radians - typically -90° for camera looking forward
            'yaw': 0.0     # radians
        }

    def publish_transform(self):
        # Create transform from camera_link to camera_optical_frame
        transform = TransformStamped()
        
        # Set header
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'bumpy7/camera_link'  # Parent frame
        transform.child_frame_id = 'camera_optical_frame'  # Child frame
        
        # Set translation (position)
        transform.transform.translation.x = self.camera_position['x']
        transform.transform.translation.y = self.camera_position['y']
        transform.transform.translation.z = self.camera_position['z']
        
        # Convert roll, pitch, yaw to quaternion
        # For camera_optical_frame, typical rotation is -90° around X axis
        # This converts from camera frame to optical frame
        from math import cos, sin
        
        # Rotation: -90° around X axis (camera looking forward)
        # This is standard for optical frames in ROS
        cy = cos(0.0 * 0.5)  # yaw
        sy = sin(0.0 * 0.5)
        cp = cos(-math.pi/2 * 0.5)  # pitch = -90° (looking forward)
        sp = sin(-math.pi/2 * 0.5)
        cr = cos(0.0 * 0.5)  # roll
        sr = sin(0.0 * 0.5)
        
        transform.transform.rotation.w = cy * cp * cr + sy * sp * sr
        transform.transform.rotation.x = cy * cp * sr - sy * sp * cr
        transform.transform.rotation.y = sy * cp * sr + cy * sp * cr
        transform.transform.rotation.z = sy * cp * cr - cy * sp * sr
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)
        
        # Log occasionally
        if hasattr(self, 'transform_count'):
            self.transform_count += 1
            if self.transform_count % 100 == 0:
                self.get_logger().info(f"Published {self.transform_count} transforms")
        else:
            self.transform_count = 1

def main(args=None):
    rclpy.init(args=args)
    node = CameraTfPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
