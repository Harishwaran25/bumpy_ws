#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('bumpy2_obstacle_avoider')

        # Publisher
        self.publisher = self.create_publisher(
            Twist,
            '/bumpy2/cmd_vel',
            10
        )

        # QoS for LaserScan (IMPORTANT FIX)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber
        self.subscription = self.create_subscription(
            LaserScan,
            '/bumpy2/scan',
            self.scan_callback,
            qos_profile
        )

        self.get_logger().info("Obstacle Avoidance Node Started")

        # Parameters (tune these if needed)
        self.safe_distance = 0.4
        self.forward_speed = 0.2
        self.turn_speed = 0.6

    def get_valid_min(self, data):
        """Filter invalid LaserScan values"""
        valid = [r for r in data if 0.1 < r < 10.0]
        return min(valid) if valid else 10.0

    def scan_callback(self, msg):
        ranges = msg.ranges
        n = len(ranges)

        # Define regions (better than equal split)
        center = n // 2

        front = self.get_valid_min(ranges[center - 20:center + 20])
        left  = self.get_valid_min(ranges[center + 20:center + 100])
        right = self.get_valid_min(ranges[center - 100:center - 20])

        # Debug log (helps tuning)
        self.get_logger().info(
            f"Front: {front:.2f}, Left: {left:.2f}, Right: {right:.2f}"
        )

        twist = Twist()

        # Decision logic
        if front < self.safe_distance:
            self.get_logger().info("Obstacle ahead → Turning")

            twist.linear.x = 0.0

            # Turn towards clearer side
            if left > right:
                twist.angular.z = self.turn_speed
            else:
                twist.angular.z = -self.turn_speed

        else:
            # Move forward
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0

        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    node = ObstacleAvoider()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
