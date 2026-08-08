#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from math import sin, cos, pi
import numpy as np

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Int32

NS_TO_SEC = 1000000000


class DiffTf(Node):

    def __init__(self):
        super().__init__("diff_tf")
        self.nodename = "diff_tf"

        #### parameters #######
        self.rate_hz = self.declare_parameter("rate_hz", 10.0).value
        self.create_timer(1.0 / self.rate_hz, self.update)

        self.ticks_meter = float(
            self.declare_parameter('ticks_meter', 2692).value)
        
        # IMPORTANT: Wheel base width = distance between wheels = 6.2 cm = 0.062 meters
        self.base_width = float(self.declare_parameter('base_width', 0.062).value)

        # Frame IDs
        self.base_frame_id = self.declare_parameter('base_frame_id',
                                                   'bumpy7/base_footprint').value
        self.odom_frame_id = self.declare_parameter('odom_frame_id',
                                                   'bumpy7/odom').value

        # Encoder wrapping parameters
        self.encoder_min = self.declare_parameter('encoder_min', -2147483648).value
        self.encoder_max = self.declare_parameter('encoder_max', 2147483648).value
        self.encoder_low_wrap = self.declare_parameter('wheel_low_wrap', (
                self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min).value
        self.encoder_high_wrap = self.declare_parameter('wheel_high_wrap', (
                self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min).value

        # FIX 1: Default publish_tf to False — EKF owns the odom→base_footprint TF.
        # If you are NOT using EKF, set this to True in your params file.
        self.publish_tf = self.declare_parameter('publish_tf', False).value

        # internal data
        self.enc_left = None
        self.enc_right = None
        self.left = 0.0
        self.right = 0.0
        self.lmult = 0.0
        self.rmult = 0.0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0.0
        self.y = 0.0
        
        # Quaternion in (x, y, z, w) format — start at identity
        self.orientation_q = np.array([0.0, 0.0, 0.0, 1.0])
        
        self.dx = 0.0
        self.dr = 0.0
        self.then = self.get_clock().now()

        # Subscriptions and publishers
        self.create_subscription(Int32, "left_encoder_ticks", self.lwheel_callback, 10)
        self.create_subscription(Int32, "right_encoder_ticks", self.rwheel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.odom_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info(
            f"DiffTf initialized: base_width={self.base_width}m, "
            f"ticks_meter={self.ticks_meter}, publish_tf={self.publish_tf}"
        )

    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions (x, y, z, w format)."""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return np.array([
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2
        ])
    
    def get_yaw(self):
        """Extract yaw angle from current orientation quaternion."""
        qx, qy, qz, qw = self.orientation_q
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return np.arctan2(siny_cosp, cosy_cosp)
    
    def quaternion_from_yaw(self, yaw):
        """Create a pure-yaw quaternion (x, y, z, w format)."""
        return np.array([0.0, 0.0, sin(yaw / 2.0), cos(yaw / 2.0)])

    def update(self):
        now = self.get_clock().now()
        elapsed = now - self.then
        self.then = now
        elapsed = elapsed.nanoseconds / NS_TO_SEC

        if elapsed <= 0:
            return

        if self.enc_left is None:
            d_left = 0.0
            d_right = 0.0
        else:
            d_left  = (self.left  - self.enc_left)  / self.ticks_meter
            d_right = (self.right - self.enc_right) / self.ticks_meter

        self.enc_left  = self.left
        self.enc_right = self.right

        # Linear and angular displacement this tick
        d  = (d_left + d_right) / 2.0
        th = (d_right - d_left) / self.base_width

        # Velocities
        self.dx = d  / elapsed
        self.dr = th / elapsed

        # FIX 2: Get current yaw BEFORE updating orientation.
        # Use the midpoint angle (current_yaw + th/2) for position integration —
        # this is the standard 2nd-order Runge-Kutta diff-drive formula and avoids
        # the double-rotation error that was in the original code.
        current_yaw = self.get_yaw()

        if d != 0:
            midpoint_yaw = current_yaw + th / 2.0
            self.x += d * cos(midpoint_yaw)
            self.y += d * sin(midpoint_yaw)

        # Update orientation quaternion AFTER position
        if th != 0:
            delta_q = self.quaternion_from_yaw(th)
            self.orientation_q = self.quaternion_multiply(self.orientation_q, delta_q)
            norm = np.linalg.norm(self.orientation_q)
            if norm > 1e-6:
                self.orientation_q = self.orientation_q / norm
            else:
                # Fallback to identity if quaternion degenerates
                self.get_logger().warn("Quaternion norm near zero — resetting to identity")
                self.orientation_q = np.array([0.0, 0.0, 0.0, 1.0])

        # Build quaternion message
        quaternion_msg = Quaternion()
        quaternion_msg.x = float(self.orientation_q[0])
        quaternion_msg.y = float(self.orientation_q[1])
        quaternion_msg.z = float(self.orientation_q[2])
        quaternion_msg.w = float(self.orientation_q[3])

        # Optionally broadcast TF (disabled by default — EKF owns this transform)
        if self.publish_tf:
            transform_stamped_msg = TransformStamped()
            transform_stamped_msg.header.stamp = now.to_msg()
            transform_stamped_msg.header.frame_id = self.odom_frame_id
            transform_stamped_msg.child_frame_id = self.base_frame_id
            transform_stamped_msg.transform.translation.x = self.x
            transform_stamped_msg.transform.translation.y = self.y
            transform_stamped_msg.transform.translation.z = 0.0
            transform_stamped_msg.transform.rotation = quaternion_msg
            self.odom_broadcaster.sendTransform(transform_stamped_msg)

        # FIX 3: Use proper diagonal covariance matrices.
        # Off-diagonal terms must be 0; unmeasured axes get large values (1e6)
        # so the EKF knows to ignore them.
        pose_cov = [0.0] * 36
        pose_cov[0]  = 0.05   # x
        pose_cov[7]  = 0.05   # y
        pose_cov[14] = 1e6    # z  (2D robot — unmeasured)
        pose_cov[21] = 1e6    # roll  (unmeasured)
        pose_cov[28] = 1e6    # pitch (unmeasured)
        pose_cov[35] = 0.1    # yaw

        twist_cov = [0.0] * 36
        twist_cov[0]  = 0.05  # vx
        twist_cov[7]  = 1e6   # vy (non-holonomic — should always be 0)
        twist_cov[14] = 1e6   # vz
        twist_cov[21] = 1e6   # wx
        twist_cov[28] = 1e6   # wy
        twist_cov[35] = 0.1   # wz

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion_msg
        odom.pose.covariance = pose_cov
        odom.twist.twist.linear.x  = self.dx
        odom.twist.twist.linear.y  = 0.0
        odom.twist.twist.angular.z = self.dr
        odom.twist.covariance = twist_cov

        self.odom_pub.publish(odom)

    def lwheel_callback(self, msg):
        enc = msg.data
        if enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap:
            self.lmult += 1
        if enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap:
            self.lmult -= 1
        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min))
        self.prev_lencoder = enc

    def rwheel_callback(self, msg):
        enc = msg.data
        if enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap:
            self.rmult += 1
        if enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap:
            self.rmult -= 1
        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc


def main(args=None):
    rclpy.init(args=args)
    try:
        diff_tf = DiffTf()
        rclpy.spin(diff_tf)
    except rclpy.exceptions.ROSInterruptException:
        pass
    finally:
        if 'diff_tf' in locals():
            diff_tf.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

