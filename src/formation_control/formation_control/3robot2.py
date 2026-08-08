#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class RobotController(Node):
    def __init__(self, robot_name):
        super().__init__(f'{robot_name}_controller')
        self.robot_name = robot_name
        self.publisher = self.create_publisher(
            Twist, 
            f'/{robot_name}/cmd_vel', 
            10
        )
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.current_state = 'forward'
        self.state_start_time = time.time()
        self.get_logger().info(f'Starting {robot_name} controller')
        
    def timer_callback(self):
        current_time = time.time()
        elapsed_time = current_time - self.state_start_time
        
        # Create a Twist message
        msg = Twist()
        
        # State machine for robot movement
        if self.current_state == 'forward':
            # Move forward - FIXED: negative value for forward movement
            msg.linear.x = -0.2  # Forward speed (reversed from previous version)
            msg.angular.z = 0.0  # No rotation
            
            if elapsed_time >= 2.0:  # After 2 seconds
                self.current_state = 'backward'
                self.state_start_time = current_time
                self.get_logger().info(f'{self.robot_name} changing to backward')
                
        elif self.current_state == 'backward':
            # Move backward - FIXED: positive value for backward movement
            msg.linear.x = 0.2  # Backward speed (reversed from previous version)
            msg.angular.z = 0.0  # No rotation
            
            if elapsed_time >= 2.0:  # After 2 seconds
                self.current_state = 'spin'
                self.state_start_time = current_time
                self.get_logger().info(f'{self.robot_name} changing to spin')
                
        elif self.current_state == 'spin':
            # Spin
            msg.linear.x = 0.0  # No linear movement
            msg.angular.z = 0.5  # Rotate
            
            if elapsed_time >= 2.0:  # After 2 seconds
                self.current_state = 'stop'
                self.state_start_time = current_time
                self.get_logger().info(f'{self.robot_name} stopping')
                
        else:  # 'stop' state
            # Stop the robot
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        
        # Publish the message
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    # Create controllers for both robots
    bumpy4_controller = RobotController('bumpy4')
    bumpy5_controller = RobotController('bumpy1')
    bumpy6_controller = RobotController('bumpy6')
    
    # Create a MultiThreadedExecutor to run both nodes concurrently
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(bumpy4_controller)
    executor.add_node(bumpy5_controller)
    executor.add_node(bumpy6_controller)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        executor.shutdown()
        bumpy4_controller.destroy_node()
        bumpy5_controller.destroy_node()
        bumpy6_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
