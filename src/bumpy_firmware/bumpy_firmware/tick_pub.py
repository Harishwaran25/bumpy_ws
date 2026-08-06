#!/usr/bin/env python3
"""
ROS2 Humble Encoder Ticks Publisher for Raspberry Pi
Publishes encoder ticks from two motors to separate ROS2 topics
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
import time

class EncoderPublisher(Node):
    def __init__(self):
        super().__init__('encoder_publisher')
        
        # Create publishers for left and right encoder ticks
        self.left_encoder_pub = self.create_publisher(
            Int32, 
            'left_encoder_ticks', 
            10)
        
        self.right_encoder_pub = self.create_publisher(
            Int32, 
            'right_encoder_ticks', 
            10)
        
        # Create timer for publishing encoder values
        self.timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Initialize GPIO
        self.gpio_initialized = False
        self.setup_gpio()
        
        # Log that the node is running
        if self.gpio_initialized:
            self.get_logger().info('Encoder Publisher Node started successfully')
        else:
            self.get_logger().warn('Encoder Publisher Node started but GPIO setup failed')
    
    def setup_gpio(self):
        """Set up GPIO for encoder reading"""
        try:
            # GPIO setup - BCM mode (DON'T cleanup here!)
            GPIO.setmode(GPIO.BCM)
            self.get_logger().info('GPIO initialized in BCM mode')
            
            # Encoder pins (BCM numbering)
            self.LEFT_ENCODER_PIN_A = 23  # Green wire for left motor
            self.LEFT_ENCODER_PIN_B = 24  # Yellow wire for left motor
            self.RIGHT_ENCODER_PIN_A = 17  # Yellow wire for right motor
            self.RIGHT_ENCODER_PIN_B = 27  # Green wire for right motor
            
            # Variables for encoder counts
            self.left_encoder_count = 0
            self.right_encoder_count = 0
            
            # Set up GPIO pins for encoders
            GPIO.setup(self.LEFT_ENCODER_PIN_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.LEFT_ENCODER_PIN_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.RIGHT_ENCODER_PIN_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.RIGHT_ENCODER_PIN_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            self.get_logger().info(f'GPIO pins configured: Left({self.LEFT_ENCODER_PIN_A},{self.LEFT_ENCODER_PIN_B}) Right({self.RIGHT_ENCODER_PIN_A},{self.RIGHT_ENCODER_PIN_B})')
            
            # Set up encoder interrupts
            try:
                GPIO.add_event_detect(
                    self.LEFT_ENCODER_PIN_A, 
                    GPIO.BOTH, 
                    callback=self.left_encoder_callback, 
                    bouncetime=1
                )
                self.get_logger().info(f'Left encoder event detection set up on pin {self.LEFT_ENCODER_PIN_A}')
                
                GPIO.add_event_detect(
                    self.RIGHT_ENCODER_PIN_A, 
                    GPIO.BOTH, 
                    callback=self.right_encoder_callback, 
                    bouncetime=1
                )
                self.get_logger().info(f'Right encoder event detection set up on pin {self.RIGHT_ENCODER_PIN_A}')
                
                self.gpio_initialized = True
                
            except RuntimeError as e:
                self.get_logger().error(f'Failed to set up encoder event detection: {e}')
                self.get_logger().error('This usually means:')
                self.get_logger().error('  1. Insufficient permissions (try: sudo usermod -a -G gpio $USER)')
                self.get_logger().error('  2. Pins already in use (try rebooting)')
                self.get_logger().error('  3. Run with sudo as a temporary test')
                
        except Exception as e:
            self.get_logger().error(f'GPIO setup failed: {type(e).__name__}: {e}')
            self.gpio_initialized = False
    
    def left_encoder_callback(self, channel):
        """Callback for left encoder interrupt"""
        a_state = GPIO.input(self.LEFT_ENCODER_PIN_A)
        b_state = GPIO.input(self.LEFT_ENCODER_PIN_B)
        
        # Logic from Arduino code
        if a_state == b_state:
            self.left_encoder_count -= 1
        else:
            self.left_encoder_count += 1
    
    def right_encoder_callback(self, channel):
        """Callback for right encoder interrupt"""
        a_state = GPIO.input(self.RIGHT_ENCODER_PIN_A)
        b_state = GPIO.input(self.RIGHT_ENCODER_PIN_B)
        
        # Logic from Arduino code
        if a_state == b_state:
            self.right_encoder_count -= 1
        else:
            self.right_encoder_count += 1
    
    def timer_callback(self):
        """Timer callback to publish encoder values"""
        # Create messages
        left_msg = Int32()
        left_msg.data = self.left_encoder_count
        
        right_msg = Int32()
        right_msg.data = self.right_encoder_count
        
        # Publish messages
        self.left_encoder_pub.publish(left_msg)
        self.right_encoder_pub.publish(right_msg)
        
        # Log current values occasionally (every 50 publishes = 5 seconds at 10Hz)
        if not hasattr(self, '_publish_count'):
            self._publish_count = 0
        self._publish_count += 1
        if self._publish_count % 50 == 0:
            self.get_logger().info(f'\nleft_encoder_value: {self.left_encoder_count}\nright_encoder_value: {self.right_encoder_count}')
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self.get_logger().info('Cleaning up GPIO')
        try:
            GPIO.cleanup()
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    
    encoder_publisher = EncoderPublisher()
    
    try:
        rclpy.spin(encoder_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up (PROPER PLACE for cleanup!)
        encoder_publisher.cleanup()
        encoder_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
