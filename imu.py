import rclpy
from rclpy.node import Node
from smbus2 import SMBus
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from sensor_msgs.msg import Imu
import math
import time
import numpy as np


def is_valid_quaternion(w, x, y, z):
    """Return True only if quaternion has no NaN/Inf and is non-zero."""
    vals = [w, x, y, z]
    if any(math.isnan(v) or math.isinf(v) for v in vals):
        return False
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    return norm > 0.01


class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_imu_node')
        
        # Initialize I2C bus
        self.bus = SMBus(1)
        self.mpu_addr = 0x68
        
        # Initialize MPU6050
        self.init_mpu()
        
        # FIX 1: Removed TransformBroadcaster for imu_link.
        # The imu_link→base_link transform is a fixed/static offset that belongs
        # in the URDF (robot_state_publisher), NOT in the IMU node.
        # Broadcasting a dynamic TF here with the IMU orientation incorrectly
        # moves the imu_link frame in the world, which corrupts the TF tree.
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)
        
        self.timer = self.create_timer(0.02, self.publish_imu_data)  # 50 Hz
        
        # Complementary filter state
        self.last_time = time.time()
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0

        # FIX 2: Startup validity gate.
        # Don't publish until the first valid orientation is computed.
        # This prevents (0,0,0,0) invalid quaternions from poisoning the EKF.
        self._imu_ready = False
        self._warmup_samples = 0
        self._warmup_required = 10  # discard first N samples while filter settles

        # FIX 3: Covariance values reflect real MPU6050 noise characteristics.
        # Using 0.1 rad² for orientation is extremely large — it makes the EKF
        # weight the IMU orientation too loosely. Tighter values are better.
        # These are tunable but are reasonable starting points for MPU6050.
        self.orientation_covariance = [
            0.01, 0.0,  0.0,
            0.0,  0.01, 0.0,
            0.0,  0.0,  0.01
        ]
        self.angular_velocity_covariance = [
            1e-4, 0.0,  0.0,
            0.0,  1e-4, 0.0,
            0.0,  0.0,  1e-4
        ]
        self.linear_acceleration_covariance = [
            1e-3, 0.0,  0.0,
            0.0,  1e-3, 0.0,
            0.0,  0.0,  1e-3
        ]
        
        self.get_logger().info('MPU6050 IMU node initialized')
        
    def init_mpu(self):
        try:
            # Reset the device
            self.bus.write_byte_data(self.mpu_addr, 0x6B, 0x80)
            time.sleep(0.1)
            
            # Wake up MPU6050
            self.bus.write_byte_data(self.mpu_addr, 0x6B, 0x00)
            time.sleep(0.1)
            
            # Clock source: PLL with X-axis gyroscope reference
            self.bus.write_byte_data(self.mpu_addr, 0x6B, 0x01)
            time.sleep(0.01)
            
            # Accelerometer: +/- 2g
            self.bus.write_byte_data(self.mpu_addr, 0x1C, 0x00)
            time.sleep(0.01)
            
            # Gyroscope: +/- 250 deg/s
            self.bus.write_byte_data(self.mpu_addr, 0x1B, 0x00)
            time.sleep(0.01)
            
            # Sample rate divider → 125 Hz
            self.bus.write_byte_data(self.mpu_addr, 0x19, 0x07)
            time.sleep(0.01)
            
            # DLPF: ~20 Hz bandwidth
            self.bus.write_byte_data(self.mpu_addr, 0x1A, 0x04)
            time.sleep(0.01)
            
            who_am_i = self.bus.read_byte_data(self.mpu_addr, 0x75)
            if who_am_i == 0x68:
                self.get_logger().info(f'MPU6050 found! WHO_AM_I: 0x{who_am_i:02x}')
            else:
                self.get_logger().warn(f'Unexpected WHO_AM_I: 0x{who_am_i:02x} (expected 0x68)')
            
            time.sleep(0.1)
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MPU6050: {str(e)}')
            raise
    
    def read_raw_data(self, addr):
        try:
            high = self.bus.read_byte_data(self.mpu_addr, addr)
            low  = self.bus.read_byte_data(self.mpu_addr, addr + 1)
            value = (high << 8) | low
            if value > 32767:
                value -= 65536
            return value
        except Exception as e:
            self.get_logger().error(
                f'Error reading register 0x{addr:02x}: {str(e)}',
                throttle_duration_sec=1
            )
            return 0
    
    def get_sensor_data(self):
        acc_x  = self.read_raw_data(0x3B) / 16384.0
        acc_y  = self.read_raw_data(0x3D) / 16384.0
        acc_z  = self.read_raw_data(0x3F) / 16384.0
        gyro_x = math.radians(self.read_raw_data(0x43) / 131.0)
        gyro_y = math.radians(self.read_raw_data(0x45) / 131.0)
        gyro_z = math.radians(self.read_raw_data(0x47) / 131.0)
        return acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z
    
    def complementary_filter(self, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Clamp dt to avoid large jumps on first call or scheduler delays
        if dt <= 0 or dt > 0.5:
            dt = 0.02

        # FIX 4: Guard against zero-gravity reads (would produce NaN in atan2).
        acc_magnitude = math.sqrt(acc_x**2 + acc_y**2 + acc_z**2)
        if acc_magnitude < 0.1:
            # Sensor is reading near-zero — skip accelerometer correction this tick
            self.roll  += gyro_x * dt
            self.pitch += gyro_y * dt
            self.yaw   += gyro_z * dt
            return

        acc_roll  = math.atan2(acc_y, acc_z)
        acc_pitch = math.atan2(-acc_x, math.sqrt(acc_y**2 + acc_z**2))
        
        alpha = 0.96
        self.roll  = alpha * (self.roll  + gyro_x * dt) + (1.0 - alpha) * acc_roll
        self.pitch = alpha * (self.pitch + gyro_y * dt) + (1.0 - alpha) * acc_pitch
        self.yaw  += gyro_z * dt
    
    def publish_imu_data(self):
        try:
            acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = self.get_sensor_data()
            self.complementary_filter(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)

            # FIX 2 (continued): Discard initial samples while the complementary
            # filter converges. Publish only after warmup is complete.
            if not self._imu_ready:
                self._warmup_samples += 1
                if self._warmup_samples < self._warmup_required:
                    return
                self._imu_ready = True
                self.get_logger().info('IMU warmup complete — starting to publish.')

            # Compute quaternion from roll/pitch/yaw
            cr = math.cos(self.roll  * 0.5)
            sr = math.sin(self.roll  * 0.5)
            cp = math.cos(self.pitch * 0.5)
            sp = math.sin(self.pitch * 0.5)
            cy = math.cos(self.yaw   * 0.5)
            sy = math.sin(self.yaw   * 0.5)

            qw = cr * cp * cy + sr * sp * sy
            qx = sr * cp * cy - cr * sp * sy
            qy = cr * sp * cy + sr * cp * sy
            qz = cr * cp * sy - sr * sp * cy

            # FIX 5: Validate quaternion before publishing.
            # A NaN or zero quaternion will immediately corrupt the EKF.
            if not is_valid_quaternion(qw, qx, qy, qz):
                self.get_logger().warn(
                    f'Invalid quaternion ({qw:.3f},{qx:.3f},{qy:.3f},{qz:.3f}) — skipping publish',
                    throttle_duration_sec=1.0
                )
                return

            imu_msg = Imu()
            imu_msg.header.stamp    = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'bumpy7/imu_link'

            imu_msg.orientation.w = float(qw)
            imu_msg.orientation.x = float(qx)
            imu_msg.orientation.y = float(qy)
            imu_msg.orientation.z = float(qz)

            for i in range(9):
                imu_msg.orientation_covariance[i]        = float(self.orientation_covariance[i])
                imu_msg.angular_velocity_covariance[i]   = float(self.angular_velocity_covariance[i])
                imu_msg.linear_acceleration_covariance[i] = float(self.linear_acceleration_covariance[i])

            imu_msg.angular_velocity.x = float(gyro_x)
            imu_msg.angular_velocity.y = float(gyro_y)
            imu_msg.angular_velocity.z = float(gyro_z)

            imu_msg.linear_acceleration.x = float(acc_x * 9.81)
            imu_msg.linear_acceleration.y = float(acc_y * 9.81)
            imu_msg.linear_acceleration.z = float(acc_z * 9.81)

            self.imu_publisher.publish(imu_msg)

            # NOTE: The imu_link TF is intentionally NOT broadcast here.
            # The static transform from base_link → imu_link is defined in the
            # URDF and published by robot_state_publisher. Broadcasting a dynamic
            # TF here (with the IMU's world orientation) was overwriting the static
            # transform and corrupting the TF tree.

        except Exception as e:
            self.get_logger().error(
                f'Error publishing IMU data: {str(e)}',
                throttle_duration_sec=1
            )


def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

