#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import glob
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class LowLatencyCameraNode(Node):
    def __init__(self):
        super().__init__('low_latency_camera_node')
        
        # QoS profile with RELIABLE for RViz compatibility
        self.qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # ROS2 Publisher
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Image,
            '/bumpy7/image_raw',
            qos_profile=self.qos_profile
        )
        
        # CameraInfo publisher for Camera display in RViz
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '/bumpy7/camera_info',
            qos_profile=self.qos_profile
        )
        
        # Auto-detect and open camera
        self.cap = self.find_and_open_camera()
        if not self.cap or not self.cap.isOpened():
            self.get_logger().error("Failed to open any USB camera!")
            raise RuntimeError("Camera initialization failed")
        
        # Low-latency configuration - Balanced FPS
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
	# In camera.py, line ~47, change from MJPEG:
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
        #self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

        # Get actual resolution after setting
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        
        self.frame = None
        
        # Log successful camera initialization
        self.get_logger().info(f"✅ Camera initialized: {actual_width}x{actual_height} @ {actual_fps}fps")
        self.get_logger().info("📡 Publishing with RELIABLE QoS")
        
        # Create CameraInfo message
        self.camera_info_msg = self.create_camera_info(actual_width, actual_height)
        
        # Publishing timer - match camera FPS
        timer_period = 1.0 / actual_fps if actual_fps > 0 else 0.033
        self.timer = self.create_timer(timer_period, self.publish_frame)
        
        self.get_logger().info(f"🎥 Publishing at {1.0/timer_period:.1f} Hz")
    
    def find_and_open_camera(self):
        """Automatically find and open the working camera device"""
        # Get all video devices
        video_devices = sorted(glob.glob('/dev/video*'))
        
        if not video_devices:
            self.get_logger().error("❌ No video devices found!")
            return None
        
        self.get_logger().info(f"🔍 Found {len(video_devices)} video devices")
        
        # Try each device
        for device in video_devices:
            self.get_logger().info(f"   Trying {device}...")
            try:
                cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
                
                if not cap.isOpened():
                    self.get_logger().info(f"      ⚠️  Cannot open {device}")
                    continue
                
                # Test if we can read frames
                ret, frame = cap.read()
                if ret and frame is not None:
                    self.get_logger().info(f"      ✅ Success! Using {device}")
                    return cap
                else:
                    self.get_logger().info(f"      ⚠️  {device} opened but cannot read frames")
                    cap.release()
                    
            except Exception as e:
                self.get_logger().info(f"      ⚠️  Error: {str(e)}")
                continue
        
        self.get_logger().error("❌ No working camera found!")
        return None
    
    def create_camera_info(self, width, height):
        """Create a basic CameraInfo message"""
        camera_info = CameraInfo()
        camera_info.header.frame_id = "bumpy7/camera_link"
        camera_info.width = width
        camera_info.height = height
        
        # Approximate camera matrix (adjust based on your camera)
        fx = fy = float(width)  # Focal length approximation
        cx = float(width) / 2.0
        cy = float(height) / 2.0
        
        camera_info.k = [fx, 0.0, cx,
                        0.0, fy, cy,
                        0.0, 0.0, 1.0]
        
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion
        
        camera_info.r = [1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0]
        
        camera_info.p = [fx, 0.0, cx, 0.0,
                        0.0, fy, cy, 0.0,
                        0.0, 0.0, 1.0, 0.0]
        
        return camera_info

    def publish_frame(self):
        ret, self.frame = self.cap.read()
        if ret and self.frame is not None:
            try:
                # Publish image
                ros_image = self.bridge.cv2_to_imgmsg(self.frame, "bgr8")
                timestamp = self.get_clock().now().to_msg()
                ros_image.header.stamp = timestamp
                ros_image.header.frame_id = "bumpy7/camera_link"
                self.publisher.publish(ros_image)
                
                # Publish camera info with same timestamp
                self.camera_info_msg.header.stamp = timestamp
                self.camera_info_pub.publish(self.camera_info_msg)
            except Exception as e:
                self.get_logger().error(f"Publish error: {str(e)}", throttle_duration_sec=1)
        else:
            self.get_logger().warn("Failed to read frame", throttle_duration_sec=2)

    def destroy_node(self):
        if self.cap and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("📷 Camera released")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LowLatencyCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down camera node...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
