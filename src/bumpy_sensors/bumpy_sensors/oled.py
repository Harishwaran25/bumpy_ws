#!/usr/bin/env python3
# ROS2 node for OLED display with rounded rectangle eyes
# Using PiU8g2 library for Arduino-like graphics on Raspberry Pi
# Save this file as oled_display_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Instead of u8g2-python, we'll use a more common library
# This will need to be installed with:
# pip3 install luma.oled
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306, sh1106
from PIL import Image, ImageDraw, ImageFont

import socket
import getpass
import time
import math
import os
import threading
import random
import sys
import tty
import termios

class OledDisplayNode(Node):
    def __init__(self):
        super().__init__('oled_display_node')
        
        # Set up OLED display using luma.oled (widely available)
        # Initialize the I2C interface
        serial = i2c(port=1, address=0x3C)  # Change address if needed (0x3C or 0x3D are common)
        
        # Choose your display type (ssd1306 is most common)
        self.device = ssd1306(serial, width=128, height=64)
        # If you have a SH1106 display, uncomment this and comment the line above:
        # self.device = sh1106(serial, width=128, height=64)
        
        # Display dimensions
        self.WIDTH = 128
        self.HEIGHT = 64
        
        # Create a font
        try:
            self.font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 12)
        except IOError:
            self.font = ImageFont.load_default()
        
        # Get system info
        self.username = getpass.getuser()
        self.ip_address = self.get_ip_address()
        
        # Subscribe to cmd_vel
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Create a timer that calls the display_callback function every 0.05 seconds (20fps)
        self.timer = self.create_timer(0.05, self.display_callback)
        
        # Animation state
        self.start_time = time.time()
        self.mode = "info"  # Start with info display
        self.blink_state = False
        self.blink_timer = 0
        
        # Smooth transition parameters
        self.target_look_direction = 0.0
        self.current_look_direction = 0.0
        self.target_running_intensity = 0.0
        self.current_running_intensity = 0.0
        
        # Eye movement state
        self.eye_mode = "normal"  # normal, running, look_left, look_right
        self.running_intensity = 0
        self.look_direction = 0  # -1 for left, 0 for center, 1 for right
        
        # Animation parameters
        self.animation_phase = 0.0
        self.eye_squash = 0.0  # For squashing eyes during running
        
        # Start keyboard input thread
        self.key_thread = threading.Thread(target=self.keyboard_input_loop)
        self.key_thread.daemon = True
        self.key_thread.start()
        
        self.get_logger().info('OLED Display Node started')
        self.get_logger().info('Controls: i=running, l=look left, r=look right, s=stop/normal')

    def get_ip_address(self):
        try:
            # Get IP address (non-loopback)
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "IP not available"
    
    def cmd_vel_callback(self, msg):
        # Process cmd_vel to set eye mode
        if abs(msg.linear.x) > 0.2:
            # Moving forward/backward - running
            self.eye_mode = "running"
            self.target_running_intensity = min(1.0, abs(msg.linear.x) / 2.0)
            self.target_look_direction = 0
        elif abs(msg.angular.z) > 0.2:
            # Turning - look left/right
            # Fix direction: positive angular.z is counter-clockwise (left turn)
            if msg.angular.z > 0:  # Left turn
                self.eye_mode = "look_left"
                # Positive for left look (fix inversion)
                self.target_look_direction = 1 * min(1.0, abs(msg.angular.z) / 1.0)
                self.target_running_intensity = 0
            else:  # Right turn
                self.eye_mode = "look_right"
                # Negative for right look (fix inversion)
                self.target_look_direction = -1 * min(1.0, abs(msg.angular.z) / 1.0)
                self.target_running_intensity = 0
        else:
            # No significant movement
            self.eye_mode = "normal"
            self.target_look_direction = 0
            self.target_running_intensity = 0
    
    def keyboard_input_loop(self):
        """Thread function to handle keyboard input"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            # Set stdin to raw mode
            tty.setraw(fd)
            self.running = True
            while self.running:
                # Check if data is available to avoid blocking
                import select
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    # Read a single character
                    ch = sys.stdin.read(1)
                    # Check for Ctrl+C (ASCII value 3)
                    if ord(ch) == 3:
                        self.get_logger().info('Ctrl+C detected, shutting down')
                        self.running = False
                        break
                    # Forward movement with i
                    elif ch == 'i':
                        # Publish cmd_vel for forward
                        self.publish_cmd_vel(0.5, 0.0)
                    # Backward movement with k
                    elif ch == 'k':
                        # Publish cmd_vel for backward
                        self.publish_cmd_vel(-0.5, 0.0)
                    # Left movement with j
                    elif ch == 'j':
                        # Publish cmd_vel for left turn
                        self.publish_cmd_vel(0.0, 0.5)
                    # Right movement with l
                    elif ch == 'l':
                        # Publish cmd_vel for right turn
                        self.publish_cmd_vel(0.0, -0.5)
                    # Stop with s
                    elif ch == 's':
                        # Publish cmd_vel for stop
                        self.publish_cmd_vel(0.0, 0.0)
                    elif ch == 'q':
                        self.get_logger().info('Quit command received, shutting down')
                        self.running = False
                        break
        except Exception as e:
            self.get_logger().error(f'Error in keyboard thread: {e}')
        finally:
            # Restore terminal settings
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            self.get_logger().info('Keyboard thread terminated')
            # Trigger shutdown
            if rclpy.ok():
                rclpy.shutdown()
    
    def publish_cmd_vel(self, linear_x, angular_z):
        """Helper function to publish cmd_vel messages"""
        # Create publisher if it doesn't exist
        if not hasattr(self, 'cmd_vel_publisher'):
            self.cmd_vel_publisher = self.create_publisher(
                Twist,
                'cmd_vel',
                10
            )
            
        # Create message
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        
        # Publish
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f'Published cmd_vel: linear.x={linear_x}, angular.z={angular_z}')
    
    def publish_cmd_vel(self, linear_x, angular_z):
        """Helper function to publish cmd_vel messages"""
        # Create publisher if it doesn't exist
        if not hasattr(self, 'cmd_vel_publisher'):
            self.cmd_vel_publisher = self.create_publisher(
                Twist,
                'cmd_vel',
                10
            )
            
        # Create message
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        
        # Publish
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f'Published cmd_vel: linear.x={linear_x}, angular.z={angular_z}')

    def display_callback(self):
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # Update animation parameters
        self.animation_phase += 0.1
        
        # Smooth transitions
        self.smooth_transitions()
        
        # Show system info for first 10 seconds
        if elapsed_time < 10.0:
            self.display_system_info()
        else:
            self.display_robot_eyes()
            
        # Update blink timer
        self.blink_timer += 0.05
        if self.blink_timer > 3.0:  # Blink every 3 seconds
            self.blink_state = True
            if self.blink_timer > 3.15:  # Blink duration 0.15 seconds
                self.blink_state = False
                self.blink_timer = 0
    
    def smooth_transitions(self):
        # Smoothly transition between states
        
        # Look direction (left/right)
        direction_diff = self.target_look_direction - self.current_look_direction
        if abs(direction_diff) > 0.01:
            self.current_look_direction += direction_diff * 0.1  # Adjust speed here
        else:
            self.current_look_direction = self.target_look_direction
            
        # Running intensity
        intensity_diff = self.target_running_intensity - self.current_running_intensity
        if abs(intensity_diff) > 0.01:
            self.current_running_intensity += intensity_diff * 0.1  # Adjust speed here
        else:
            self.current_running_intensity = self.target_running_intensity
            
        # Update eye squash based on running
        if self.eye_mode == "running":
            # Calculate vertical squash for running animation (bobbing up and down)
            self.eye_squash = math.sin(self.animation_phase * 1.5) * 0.2 * self.current_running_intensity
        else:
            # Gradually return to normal
            self.eye_squash *= 0.8

    def display_system_info(self):
        with canvas(self.device) as draw:
            # Draw a border
            draw.rectangle(self.device.bounding_box, outline="white", fill="black")
            
            # Display username
            draw.text((5, 5), f"User: {self.username}", fill="white", font=self.font)
            
            # Display IP address
            draw.text((5, 25), f"IP: {self.ip_address}", fill="white", font=self.font)

    def display_robot_eyes(self):
        with canvas(self.device) as draw:
            # Apply eye movement based on mode
            x_offset = 0
            y_offset = 0
            
            if self.current_running_intensity > 0.05:
                # Random jitter for running + some vertical movement
                x_offset = random.uniform(-3, 3) * self.current_running_intensity
                y_offset = random.uniform(-2, 2) * self.current_running_intensity
                y_offset += math.sin(self.animation_phase * 1.5) * 3 * self.current_running_intensity
            
            # Draw two cartoon eyes with the applied offset
            self.draw_eye(draw, 40 + x_offset, 32 + y_offset, 30, self.current_look_direction)  # Left eye
            self.draw_eye(draw, 88 + x_offset, 32 + y_offset, 30, self.current_look_direction)  # Right eye
    
    def rounded_rectangle(self, draw, xy, corner_radius, outline=None, fill=None):
        """Draw a rounded rectangle with PIL"""
        x1, y1, x2, y2 = xy
        radius = int(corner_radius)
        
        # Draw the middle part
        draw.rectangle([x1, y1 + radius, x2, y2 - radius], fill=fill, outline=outline)
        draw.rectangle([x1 + radius, y1, x2 - radius, y2], fill=fill, outline=outline)
        
        # Draw the four corners
        draw.pieslice([x1, y1, x1 + radius * 2, y1 + radius * 2], 180, 270, fill=fill, outline=outline)
        draw.pieslice([x2 - radius * 2, y1, x2, y1 + radius * 2], 270, 0, fill=fill, outline=outline)
        draw.pieslice([x1, y2 - radius * 2, x1 + radius * 2, y2], 90, 180, fill=fill, outline=outline)
        draw.pieslice([x2 - radius * 2, y2 - radius * 2, x2, y2], 0, 90, fill=fill, outline=outline)
    
    def draw_eye(self, draw, x, y, size, look_direction):
        # Convert to integers for drawing
        x, y = int(x), int(y)
        
        # Apply vertical squash for running effect
        height_adjust = int(size * self.eye_squash)
        width_adjust = int(size * 0.1 * abs(self.eye_squash))  # Compensate width slightly
        
        # Calculate eye dimensions
        eye_width = size + width_adjust
        eye_height = size - height_adjust
        
        # Eye coordinates for rounded rectangle
        x1 = x - eye_width // 2
        y1 = y - eye_height // 2
        x2 = x + eye_width // 2
        y2 = y + eye_height // 2
        
        # Calculate corner radius for rounded rectangle
        corner_radius = min(10, eye_height // 3)
        
        # Draw eye outline as a rounded rectangle
        self.rounded_rectangle(draw, (x1, y1, x2, y2), corner_radius, outline="white", fill="white")
        
        # Base pupil movement
        elapsed = time.time() - self.start_time
        base_x_offset = math.sin(elapsed * 0.5) * 3
        base_y_offset = math.cos(elapsed * 0.3) * 2
        
        # Add directional looking
        look_x_offset = look_direction * size * 0.3  # Look left/right
        
        # Pupil (inner circle) - position depends on mode
        pupil_size = size // 2
        pupil_x = x + int(base_x_offset + look_x_offset)
        pupil_y = y + int(base_y_offset)
        
        # Constrain pupil position to stay within eye
        max_x_offset = eye_width // 2 - pupil_size // 2 - 2
        max_y_offset = eye_height // 2 - pupil_size // 2 - 2
        pupil_x = min(max(pupil_x, x - max_x_offset), x + max_x_offset)
        pupil_y = min(max(pupil_y, y - max_y_offset), y + max_y_offset)
        
        # Don't draw pupil when blinking
        if not self.blink_state:
            # Draw filled circle for pupil (black)
            draw.ellipse(
                (pupil_x-pupil_size//2, pupil_y-pupil_size//2, 
                 pupil_x+pupil_size//2, pupil_y+pupil_size//2), 
                outline="black", fill="black"
            )
        else:
            # Draw closed eye (just a line)
            draw.line((x1+corner_radius, y, x2-corner_radius, y), fill="black", width=4)

def main(args=None):
    rclpy.init(args=args)
    node = OledDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clear display before shutting down
        node.device.clear()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()