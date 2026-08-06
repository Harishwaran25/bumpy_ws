#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except (ImportError, RuntimeError) as e:
    GPIO_AVAILABLE = False
    print(f"⚠️  GPIO not available: {e}")
import time

class GasSensorNode(Node):
    def __init__(self):
        super().__init__('gas_sensor_node')
        
        self.sensors_enabled = False
        
        # Try to setup GPIO
        if GPIO_AVAILABLE:
            try:
                # GPIO Setup
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                
                # Ultrasonic Sensor Pins (HC-SR04)
                self.TRIG = 22
                self.ECHO = 25
                GPIO.setup(self.TRIG, GPIO.OUT)
                GPIO.setup(self.ECHO, GPIO.IN)
                
                # MQ2 Gas Sensor Digital Pin (No ADC needed)
                self.MQ2_PIN = 26 # Change to your GPIO pin if different
                GPIO.setup(self.MQ2_PIN, GPIO.IN)
                
                self.sensors_enabled = True
                self.get_logger().info("✅ GPIO initialized successfully")
                
            except Exception as e:
                self.get_logger().error(f"❌ GPIO setup failed: {str(e)}")
                self.get_logger().warn("⚠️  Node running in SIMULATION mode (no hardware)")
                self.sensors_enabled = False
        else:
            self.get_logger().warn("⚠️  RPi.GPIO not available - running in SIMULATION mode")
        
        # Publishers (always create, even without hardware)
        self.ultrasonic_pub = self.create_publisher(
            Range, 
            '/bumpy7/ultrasonic', 
            10
        )
        self.gas_alarm_pub = self.create_publisher(
            Bool, 
            '/bumpy7/gas_alarm', 
            10
        )
        self.hole_pub = self.create_publisher(
            PoseStamped, 
            '/bumpy7/hole_detected', 
            10
        )
        
        # Subscribe to robot pose for hole location tracking
        self.current_pose = None
        self.create_subscription(
            PoseStamped, 
            '/bumpy7/pose', 
            self.pose_callback, 
            10
        )
        
        # Gas detection state
        self.gas_detected = False
        self.last_gas_state = False
        
        # Timers (always run)
        self.create_timer(0.1, self.read_ultrasonic)  # 10Hz - Fast for hole detection
        self.create_timer(0.2, self.read_gas_sensor)  # 5Hz - Gas sensor
        
        self.get_logger().info("=" * 50)
        if self.sensors_enabled:
            self.get_logger().info("✅ Gas Sensor Node Started - HARDWARE MODE")
            self.get_logger().info("MQ2 Sensor (LPG/Propane/Smoke) on GPIO 26")
            self.get_logger().info("HC-SR04 Ultrasonic on GPIO 22/25")
        else:
            self.get_logger().info("⚠️  Gas Sensor Node Started - SIMULATION MODE")
            self.get_logger().info("Publishing dummy data (sensors not connected)")
        self.get_logger().info("=" * 50)
    
    def pose_callback(self, msg):
        """Store current robot position for hole location tracking"""
        self.current_pose = msg
    
    def read_gas_sensor(self):
        """Read MQ2 gas sensor digital output"""
        try:
            if self.sensors_enabled:
                # Read digital pin (LOW = gas detected, HIGH = no gas)
                # MQ2 D0 is active LOW: 0 = gas detected, 1 = no gas
                pin_value = GPIO.input(self.MQ2_PIN)
                self.gas_detected = (pin_value == 0)
            else:
                # Simulation mode - no gas detected
                self.gas_detected = False
            
            # Publish alarm status
            alarm_msg = Bool()
            alarm_msg.data = self.gas_detected
            self.gas_alarm_pub.publish(alarm_msg)
            
            # Log only when state changes (avoid spam)
            if self.gas_detected and not self.last_gas_state:
                self.get_logger().warn("🔥 GAS LEAK DETECTED! Flammable gas present!")
            elif not self.gas_detected and self.last_gas_state:
                self.get_logger().info("✅ Gas cleared - area safe")
            
            self.last_gas_state = self.gas_detected
            
        except Exception as e:
            self.get_logger().error(f"Gas sensor error: {str(e)}", throttle_duration_sec=5)
    
    def read_ultrasonic(self):
        """Read HC-SR04 ultrasonic sensor for hole detection"""
        try:
            if not self.sensors_enabled:
                # Simulation mode - publish dummy data
                range_msg = Range()
                range_msg.header.stamp = self.get_clock().now().to_msg()
                range_msg.header.frame_id = "bumpy7/ultrasonic_link"
                range_msg.radiation_type = Range.ULTRASOUND
                range_msg.field_of_view = 0.26
                range_msg.min_range = 0.02
                range_msg.max_range = 4.0
                range_msg.range = 0.25  # 25cm - no hole
                self.ultrasonic_pub.publish(range_msg)
                return
            
            # Send 10μs trigger pulse
            GPIO.output(self.TRIG, True)
            time.sleep(0.00001)
            GPIO.output(self.TRIG, False)
            
            # Wait for echo start (with timeout)
            pulse_start = time.time()
            timeout = pulse_start + 0.1  # 100ms timeout
            
            while GPIO.input(self.ECHO) == 0 and time.time() < timeout:
                pulse_start = time.time()
            
            # Wait for echo end (with timeout)
            pulse_end = time.time()
            timeout = pulse_end + 0.1
            
            while GPIO.input(self.ECHO) == 1 and time.time() < timeout:
                pulse_end = time.time()
            
            # Calculate distance
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150  # Speed of sound = 343m/s, /2 for round trip
            distance = round(distance, 2)
            
            # Publish range message
            range_msg = Range()
            range_msg.header.stamp = self.get_clock().now().to_msg()
            range_msg.header.frame_id = "bumpy7/ultrasonic_link"
            range_msg.radiation_type = Range.ULTRASOUND
            range_msg.field_of_view = 0.26  # ~15 degrees cone
            range_msg.min_range = 0.02      # 2cm minimum
            range_msg.max_range = 4.0       # 400cm maximum
            range_msg.range = distance / 100.0  # Convert to meters
            self.ultrasonic_pub.publish(range_msg)
            
            # Detect hole (distance > 50cm = ground dropped away)
            if distance > 50 and distance < 400:  # Valid range
                self.detect_hole(distance)
            
        except Exception as e:
            self.get_logger().error(f"Ultrasonic sensor error: {str(e)}", throttle_duration_sec=5)
    
    def detect_hole(self, distance):
        """Publish hole detection event with location and gas status"""
        if self.current_pose:
            hole_msg = PoseStamped()
            hole_msg.header.stamp = self.get_clock().now().to_msg()
            hole_msg.header.frame_id = "map"
            hole_msg.pose = self.current_pose.pose
            self.hole_pub.publish(hole_msg)
            
            # Log with gas status
            gas_status = "WITH GAS LEAK! ⚠️" if self.gas_detected else "(no gas detected)"
            self.get_logger().info(
                f"🕳️  HOLE DETECTED at depth: {distance}cm {gas_status}"
            )
    
    def destroy_node(self):
        """Clean up GPIO on shutdown"""
        self.get_logger().info("Shutting down sensor node...")
        if self.sensors_enabled and GPIO_AVAILABLE:
            GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GasSensorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()