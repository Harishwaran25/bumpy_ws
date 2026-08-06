import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class LeaderElectionNode(Node):
    def __init__(self):
        super().__init__('leader_election_node')
        
        # Store status for both robots
        self.robot_status = {
            'bumpy7': {
                'battery': None, 
                'covariance': None, 
                'last_update': None,
                'score': 0.0
            },
            'bumpy5': {
                'battery': None, 
                'covariance': None, 
                'last_update': None,
                'score': 0.0
            }
        }
        
        self.current_leader = None
        self.current_follower = None
        
        # Subscribe to both robots' status topics
        self.create_subscription(
            String,
            '/bumpy7/robot_status',
            lambda msg: self.status_callback(msg, 'bumpy7'),
            10
        )
        
        self.create_subscription(
            String,
            '/bumpy5/robot_status',
            lambda msg: self.status_callback(msg, 'bumpy5'),
            10
        )
        
        # Publisher for leader election result
        self.leader_publisher = self.create_publisher(String, '/leader_ele', 10)
        
        # Timer to periodically evaluate and assign roles
        self.timer = self.create_timer(2.0, self.evaluate_and_publish_leader)
        
        self.get_logger().info("="*60)
        self.get_logger().info("Leader Election Node Started")
        self.get_logger().info("Monitoring: /bumpy7/robot_status and /bumpy5/robot_status")
        self.get_logger().info("Publishing to: /leader_ele")
        self.get_logger().info("="*60)
    
    def status_callback(self, msg, robot_name):
        """Process incoming robot status messages"""
        try:
            data = json.loads(msg.data)
            self.robot_status[robot_name]['battery'] = data.get('battery')
            self.robot_status[robot_name]['covariance'] = data.get('covariance')
            self.robot_status[robot_name]['last_update'] = self.get_clock().now()
            
        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to parse status from {robot_name}")
    
    def calculate_score(self, battery, covariance):
        """
        Calculate robot fitness score
        Higher score = better suited to be leader
        
        Scoring criteria:
        - Battery: 70% weight (higher is better)
        - Localization: 30% weight (lower covariance is better)
        """
        if battery is None or covariance is None:
            return -1  # Invalid score for robots without data
        
        # Normalize battery (0-100 range)
        battery_score = battery
        
        # Normalize covariance (invert so lower covariance = higher score)
        # Adjust max_acceptable_cov based on your environment
        max_acceptable_cov = 5.0
        if covariance > max_acceptable_cov:
            localization_score = 0
        else:
            localization_score = (1 - (covariance / max_acceptable_cov)) * 100
            localization_score = max(0, localization_score)
        
        # Weighted combination: 70% battery, 30% localization
        total_score = (0.7 * battery_score) + (0.3 * localization_score)
        
        return total_score
    
    def evaluate_and_publish_leader(self):
        """Evaluate robots and publish leader election result"""
        
        # Check if we have data from both robots
        bumpy7_data = self.robot_status['bumpy7']
        bumpy5_data = self.robot_status['bumpy5']
        
        # Check for stale data (no updates in last 5 seconds)
        current_time = self.get_clock().now()
        timeout_duration = rclpy.duration.Duration(seconds=5.0)
        
        bumpy7_active = (bumpy7_data['last_update'] is not None and 
                        (current_time - bumpy7_data['last_update']) < timeout_duration)
        bumpy5_active = (bumpy5_data['last_update'] is not None and 
                        (current_time - bumpy5_data['last_update']) < timeout_duration)
        
        if not bumpy7_active and not bumpy5_active:
            self.get_logger().warn("⚠️  No data from either robot", throttle_duration_sec=5.0)
            return
        
        # Calculate scores
        score_bumpy7 = self.calculate_score(
            bumpy7_data['battery'], 
            bumpy7_data['covariance']
        ) if bumpy7_active else -1
        
        score_bumpy5 = self.calculate_score(
            bumpy5_data['battery'], 
            bumpy5_data['covariance']
        ) if bumpy5_active else -1
        
        # Store scores
        self.robot_status['bumpy7']['score'] = score_bumpy7
        self.robot_status['bumpy5']['score'] = score_bumpy5
        
        # Determine leader and follower
        if not bumpy7_active and bumpy5_active:
            leader = 'bumpy5'
            follower = 'bumpy7'
        elif not bumpy5_active and bumpy7_active:
            leader = 'bumpy7'
            follower = 'bumpy5'
        elif score_bumpy7 > score_bumpy5:
            leader = 'bumpy7'
            follower = 'bumpy5'
        elif score_bumpy5 > score_bumpy7:
            leader = 'bumpy5'
            follower = 'bumpy7'
        else:
            # Tie - default to bumpy7 as leader
            leader = 'bumpy7'
            follower = 'bumpy5'
        
        # Check if leader changed
        leader_changed = (self.current_leader != leader)
        self.current_leader = leader
        self.current_follower = follower
        
        # Prepare leader election data for dashboard
        leader_election_data = {
            "leader": leader,
            "follower": follower,
            "bumpy7": {
                "battery": round(bumpy7_data['battery'], 2) if bumpy7_data['battery'] is not None else 0.0,
                "covariance": round(bumpy7_data['covariance'], 4) if bumpy7_data['covariance'] is not None else 999.0,
                "score": round(score_bumpy7, 2) if score_bumpy7 > 0 else 0.0,
                "active": bumpy7_active
            },
            "bumpy5": {
                "battery": round(bumpy5_data['battery'], 2) if bumpy5_data['battery'] is not None else 0.0,
                "covariance": round(bumpy5_data['covariance'], 4) if bumpy5_data['covariance'] is not None else 999.0,
                "score": round(score_bumpy5, 2) if score_bumpy5 > 0 else 0.0,
                "active": bumpy5_active
            },
            "timestamp": self.get_clock().now().to_msg().sec
        }
        
        # Publish to /leader_ele topic for dashboard
        msg = String()
        msg.data = json.dumps(leader_election_data)
        self.leader_publisher.publish(msg)
        
        # Log decision with visual formatting
        if leader_changed:
            self.get_logger().info("="*60)
            self.get_logger().info("🔄 LEADER CHANGE DETECTED!")
            self.get_logger().info("="*60)
        
        self.get_logger().info(
            f"👑 LEADER: {leader} (Score: {score_bumpy7 if leader == 'bumpy7' else score_bumpy5:.2f}) | "
            f"👥 FOLLOWER: {follower} (Score: {score_bumpy5 if follower == 'bumpy5' else score_bumpy7:.2f})"
        )
        self.get_logger().info(
            f"   📊 Bumpy7 - Battery: {bumpy7_data['battery']:.1f}%, Cov: {bumpy7_data['covariance']:.4f}, Active: {bumpy7_active}"
        )
        self.get_logger().info(
            f"   📊 Bumpy5 - Battery: {bumpy5_data['battery']:.1f}%, Cov: {bumpy5_data['covariance']:.4f}, Active: {bumpy5_active}"
        )
        self.get_logger().info("-"*60)

def main(args=None):
    rclpy.init(args=args)
    node = LeaderElectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
