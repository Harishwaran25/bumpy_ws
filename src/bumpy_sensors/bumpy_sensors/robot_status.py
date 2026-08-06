import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
import json

class Bumpy7StatusNode(Node):
    def __init__(self):
        super().__init__('bumpy7_status_node')
        
        self.robot_name = "bumpy7"
        
        # Battery simulation parameters
        self.battery_percentage = 100.0  # Start at 100%
        self.discharge_rate = 0.05  # 0.05% per second (slower discharge)
        self.voltage_min = 9.0   # Minimum safe voltage (cutoff)
        self.voltage_nominal = 11.1  # Nominal voltage
        self.voltage_max = 12.6  # Maximum voltage (fully charged)
        self.battery_capacity_ah = 8.0  # 8Ah capacity
        
        # Initialize covariance
        self.covariance = None
        
        # Subscribe to AMCL pose for real-time localization confidence
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )
        
        # Publisher for simulated battery state
        self.battery_publisher = self.create_publisher(
            BatteryState, 
            '/bumpy7/battery_state', 
            10
        )
        
        # Publisher for robot status
        self.status_publisher = self.create_publisher(
            String, 
            '/bumpy7/robot_status', 
            10
        )
        
        # Timers
        self.battery_timer = self.create_timer(1.0, self.publish_battery)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Bumpy7 Status Node Started")
        self.get_logger().info(f"Battery: {self.voltage_nominal}V nominal, {self.voltage_max}V max, {self.battery_capacity_ah}Ah")
        self.get_logger().info(f"Discharge rate: {self.discharge_rate}% per second")
        self.get_logger().info("Waiting for /bumpy7/amcl_pose for real-time localization...")
        self.get_logger().info("=" * 60)
    
    def publish_battery(self):
        """Simulate and publish battery state"""
        # Simulate discharge
        self.battery_percentage -= self.discharge_rate
        
        # Reset to 100% when depleted (for continuous testing)
        if self.battery_percentage < 0:
            self.battery_percentage = 100.0
            self.get_logger().info("🔋 Battery reset to 100% (simulation)")
        
        # Calculate voltage based on percentage
        # Linear interpolation between min and max voltage
        voltage_range = self.voltage_max - self.voltage_min
        voltage = self.voltage_min + (self.battery_percentage / 100.0) * voltage_range
        
        # Simulate current draw (estimated based on typical robot consumption)
        # At 100%: ~1A, decreases as battery depletes
        estimated_current = (self.battery_percentage / 100.0) * 1.5  # 0 to 1.5A
        
        # Calculate remaining capacity
        remaining_capacity = (self.battery_percentage / 100.0) * self.battery_capacity_ah
        
        # Create and publish BatteryState message
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "bumpy7_battery"
        
        msg.voltage = voltage
        msg.current = -estimated_current  # Negative for discharging
        msg.charge = remaining_capacity  # Ah
        msg.capacity = self.battery_capacity_ah  # Ah
        msg.design_capacity = self.battery_capacity_ah
        msg.percentage = self.battery_percentage / 100.0  # 0.0 to 1.0
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        
        self.battery_publisher.publish(msg)
    
    def amcl_callback(self, msg):
        """Calculate localization confidence from AMCL covariance (real-time)"""
        cov_x = msg.pose.covariance[0]    # X position variance
        cov_y = msg.pose.covariance[7]    # Y position variance
        cov_theta = msg.pose.covariance[35]  # Orientation variance
        
        # Sum of covariances (lower is better)
        self.covariance = cov_x + cov_y + cov_theta
    
    def publish_status(self):
        """Publish robot status"""
        if self.covariance is None:
            self.get_logger().warn("No AMCL data received yet", throttle_duration_sec=10.0)
            covariance_value = 999.0
        else:
            covariance_value = self.covariance
        
        # Calculate voltage for display
        voltage_range = self.voltage_max - self.voltage_min
        voltage = self.voltage_min + (self.battery_percentage / 100.0) * voltage_range
        
        data = {
            "robot": self.robot_name,
            "battery": round(self.battery_percentage, 2),
            "voltage": round(voltage, 2),
            "covariance": round(covariance_value, 4),
            "available": True,
            "timestamp": self.get_clock().now().to_msg().sec
        }
        
        msg = String()
        msg.data = json.dumps(data)
        self.status_publisher.publish(msg)
        
        # Log every 5 seconds
        self.get_logger().info(
            f"Bumpy7 - Battery: {self.battery_percentage:.1f}% ({voltage:.2f}V), "
            f"Covariance: {covariance_value:.4f}",
            throttle_duration_sec=5.0
        )

def main(args=None):
    rclpy.init(args=args)
    node = Bumpy7StatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
