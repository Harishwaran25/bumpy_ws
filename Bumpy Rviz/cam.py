#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import threading

class LowLatencyCameraNode(Node):
    def __init__(self):
        super().__init__('low_latency_camera_node')
        
        # QoS profile
        self.qos_profile = QoSProfile(
            depth=2,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # ROS2 Publishers
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Image,
            '/bumpy7/image_raw',
            qos_profile=self.qos_profile
        )
        
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '/bumpy7/camera_info',
            qos_profile=self.qos_profile
        )
        
        # Camera Setup
        self.setup_camera()
        
        # Threading for continuous capture
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.running = True
        self.frame_count = 0
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()
        
        # Publishing timer - 30Hz
        timer_period = 1.0 / 30.0
        self.timer = self.create_timer(timer_period, self.publish_frame)
        
        # Debug timer - log stats every 5 seconds
        self.debug_timer = self.create_timer(5.0, self.debug_callback)
        
        self.get_logger().info("Low-latency USB camera node started")
    
    def setup_camera(self):
        """Initialize camera with optimal settings"""
        self.cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera on /dev/video0!")
            raise RuntimeError("Camera initialization failed")
        
        # Camera settings
        width, height = 640, 480
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        
        # Enable AUTO exposure initially
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)  # 3 = Auto mode
        
        # Get actual settings
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        
        self.get_logger().info(f"Camera configured: {actual_width}x{actual_height} @ {actual_fps}fps")
        
        # Test read a frame
        ret, test_frame = self.cap.read()
        if ret and test_frame is not None:
            self.get_logger().info(f"Test frame captured: shape={test_frame.shape}, "
                                  f"min={test_frame.min()}, max={test_frame.max()}, "
                                  f"mean={test_frame.mean():.1f}")
            if test_frame.mean() < 10:
                self.get_logger().warn("Frame is very dark! Check lighting or exposure settings")
        else:
            self.get_logger().error("Failed to capture test frame!")
        
        # Create camera info message
        self.camera_info_msg = self.create_camera_info(actual_width, actual_height)
    
    def capture_loop(self):
        """Continuously capture frames in separate thread"""
        while self.running and rclpy.ok():
            ret, frame = self.cap.read()
            if ret and frame is not None:
                with self.frame_lock:
                    self.latest_frame = frame
                    self.frame_count += 1
            else:
                self.get_logger().warn("Frame capture failed", throttle_duration_sec=5)
    
    def create_camera_info(self, width, height):
        """Create CameraInfo message"""
        camera_info = CameraInfo()
        camera_info.header.frame_id = "bumpy7/camera_link"
        camera_info.width = width
        camera_info.height = height
        
        # Approximate camera intrinsics
        fx = fy = float(width)
        cx = float(width) / 2.0
        cy = float(height) / 2.0
        
        camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        return camera_info

    def publish_frame(self):
        """Publish latest captured frame"""
        with self.frame_lock:
            if self.latest_frame is None:
                return
            frame = self.latest_frame.copy()
        
        try:
            # Convert to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            timestamp = self.get_clock().now().to_msg()
            ros_image.header.stamp = timestamp
            ros_image.header.frame_id = "bumpy7/camera_link"
            
            # Publish image
            self.publisher.publish(ros_image)
            
            # Publish camera info with same timestamp
            self.camera_info_msg.header.stamp = timestamp
            self.camera_info_pub.publish(self.camera_info_msg)
            
        except Exception as e:
            self.get_logger().error(f"Publish error: {e}", throttle_duration_sec=1)
    
    def debug_callback(self):
        """Log debug info periodically"""
        with self.frame_lock:
            if self.latest_frame is not None:
                mean_brightness = self.latest_frame.mean()
                self.get_logger().info(
                    f"Stats: {self.frame_count} frames captured, "
                    f"brightness: {mean_brightness:.1f}"
                )
            else:
                self.get_logger().warn("No frames captured yet!")

    def destroy_node(self):
        """Clean up resources"""
        self.running = False
        if self.capture_thread.is_alive():
            self.capture_thread.join(timeout=1.0)
        self.cap.release()
        self.get_logger().info("Camera resources released")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LowLatencyCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
