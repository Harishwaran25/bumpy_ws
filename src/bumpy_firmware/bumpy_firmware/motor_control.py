#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

class L298NController(Node):
    def __init__(self):
        super().__init__('l298n_controller')
        
        self.LEFT_EN = 26       
        self.RIGHT_EN = 22      
        self.LEFT_BACKWARD = 6
        self.LEFT_FORWARD = 5
        self.RIGHT_FORWARD = 20
        self.RIGHT_BACKWARD = 16
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        GPIO.setup(self.LEFT_EN, GPIO.OUT)
        GPIO.setup(self.RIGHT_EN, GPIO.OUT)
        GPIO.setup(self.LEFT_FORWARD, GPIO.OUT)
        GPIO.setup(self.LEFT_BACKWARD, GPIO.OUT)
        GPIO.setup(self.RIGHT_FORWARD, GPIO.OUT)
        GPIO.setup(self.RIGHT_BACKWARD, GPIO.OUT)
        
        self.left_pwm = GPIO.PWM(self.LEFT_EN, 100)
        self.right_pwm = GPIO.PWM(self.RIGHT_EN, 100)
        
        self.left_pwm.start(0)
        self.right_pwm.start(0)
        
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.stop_motors()
    
    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        left_speed, right_speed = self.calculate_wheel_speeds(linear_x, angular_z)
        
        self.set_motor_speeds(left_speed, right_speed)
    
    def calculate_wheel_speeds(self, linear_x, angular_z):
        wheel_separation = 0.07
        
        amplification = 3.0
        
        boosted_linear = linear_x * amplification
        boosted_angular = angular_z * amplification
        
        boosted_linear = boosted_linear  # removed negation
        boosted_angular = boosted_angular
        
        left_speed = boosted_linear - (boosted_angular * wheel_separation / 2.0)
        right_speed = boosted_linear + (boosted_angular * wheel_separation / 2.0)
        
        min_threshold = 35
        
        if abs(left_speed) > 0 and abs(left_speed) < min_threshold:
            left_speed = min_threshold if left_speed > 0 else -min_threshold
            
        if abs(right_speed) > 0 and abs(right_speed) < min_threshold:
            right_speed = min_threshold if right_speed > 0 else -min_threshold
        
        left_speed = max(min(left_speed, 100.0), -100.0)
        right_speed = max(min(right_speed, 100.0), -100.0)
        
        return left_speed, right_speed
    
    def set_motor_speeds(self, left_speed, right_speed):
        min_threshold = 30
        
        if left_speed > 0:
            GPIO.output(self.LEFT_FORWARD, GPIO.HIGH)
            GPIO.output(self.LEFT_BACKWARD, GPIO.LOW)
            actual_speed = max(abs(left_speed), min_threshold) if abs(left_speed) > 0 else 0
            self.left_pwm.ChangeDutyCycle(actual_speed)
        elif left_speed < 0:
            GPIO.output(self.LEFT_FORWARD, GPIO.LOW)
            GPIO.output(self.LEFT_BACKWARD, GPIO.HIGH)
            actual_speed = max(abs(left_speed), min_threshold) if abs(left_speed) > 0 else 0
            self.left_pwm.ChangeDutyCycle(actual_speed)
        else:
            GPIO.output(self.LEFT_FORWARD, GPIO.LOW)
            GPIO.output(self.LEFT_BACKWARD, GPIO.LOW)
            self.left_pwm.ChangeDutyCycle(0)
        
        if right_speed > 0:
            GPIO.output(self.RIGHT_FORWARD, GPIO.HIGH)
            GPIO.output(self.RIGHT_BACKWARD, GPIO.LOW)
            actual_speed = max(abs(right_speed), min_threshold) if abs(right_speed) > 0 else 0
            self.right_pwm.ChangeDutyCycle(actual_speed)
        elif right_speed < 0:
            GPIO.output(self.RIGHT_FORWARD, GPIO.LOW)
            GPIO.output(self.RIGHT_BACKWARD, GPIO.HIGH)
            actual_speed = max(abs(right_speed), min_threshold) if abs(right_speed) > 0 else 0
            self.right_pwm.ChangeDutyCycle(actual_speed)
        else:
            GPIO.output(self.RIGHT_FORWARD, GPIO.LOW)
            GPIO.output(self.RIGHT_BACKWARD, GPIO.LOW)
            self.right_pwm.ChangeDutyCycle(0)
    
    def stop_motors(self):
        GPIO.output(self.LEFT_FORWARD, GPIO.LOW)
        GPIO.output(self.LEFT_BACKWARD, GPIO.LOW)
        GPIO.output(self.RIGHT_FORWARD, GPIO.LOW)
        GPIO.output(self.RIGHT_BACKWARD, GPIO.LOW)
        self.left_pwm.ChangeDutyCycle(0)
        self.right_pwm.ChangeDutyCycle(0)
    
    def cleanup(self):
        self.stop_motors()
        self.left_pwm.stop()
        self.right_pwm.stop()
        GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)
    
    l298n_controller = L298NController()
    
    try:
        rclpy.spin(l298n_controller)
    except KeyboardInterrupt:
        pass
    finally:
        l298n_controller.cleanup()
        l298n_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
