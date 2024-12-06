import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

class RunAndStop(Node):
    def __init__(self):
        super().__init__('run_and_stop')
        
        # Publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscriber for laser scan data
        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.SYSTEM_DEFAULT)
        )
        
        # Timer for periodic control
        self.timer_period = 0.1  # Control loop period (0.1 seconds)
        self.timer = self.create_timer(self.timer_period, self.motion)
        
        # Variables for laser data and velocity commands
        self.laser_forward = float('inf')  # Default to no obstacle in front
        self.cmd = Twist()

    def laser_callback(self, msg):
        # Save the minimum frontal laser scan distance (typically at 0°)
        self.laser_forward = msg.ranges[0]

    def motion(self):
        # Log the front laser distance for debugging
        self.get_logger().info(f'Front Laser Distance: {self.laser_forward:.2f} m')

        # Check if there's an obstacle within 0.5 meters
        if self.laser_forward < 0.5:
            # Stop the robot
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Obstacle detected! Stopping.')
            return

        # Move forward if no obstacle is detected
        self.cmd.linear.x = 0.2  # Forward speed (m/s)
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        self.get_logger().info('Moving forward.')

def main(args=None):
    # Initialize the ROS communication
    rclpy.init(args=args)
    
    # Instantiate the node
    run_and_stop = RunAndStop()
    
    # Spin the node to keep it running
    rclpy.spin(run_and_stop)
    
    # Destroy the node explicitly
    run_and_stop.destroy_node()
    
    # Shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
