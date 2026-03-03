# Importing ROS2 Python client library for node creation and ROS2 communication
import rclpy
# Importing the base Node class from ROS2
from rclpy.node import Node
# Importing the LaserScan message type to receive data from the LiDAR sensor
from sensor_msgs.msg import LaserScan
# Importing Bool and Float32MultiArray standard message types
from std_msgs.msg import Bool, Float32MultiArray
# Importing math module for angle conversions and trigonometry
import math
# Importing time module for basic timing operations
import time

# Defining the SLLidarClient class to process LiDAR data and detect obstacles
class SLLidarClient(Node):
    # Initializing the obstacle_detection node
    def __init__(self):
        super().__init__('obstacle_detection')  # Naming the node 'obstacle_detection'

        # Creating a subscription to the LiDAR data topic ('scan')
        self.subscription1 = self.create_subscription(
            LaserScan,       # Message type used for LiDAR scans
            'scan',          # Topic name
            self.scan_callback,  # Callback function on message receipt
            10               # Queue size
        )

        # Creating a subscription to the 'Completed' topic to re-enable processing after a task
        self.subscription2 = self.create_subscription(
            Bool,            # Message type indicating task completion
            'Completed',     # Topic name
            self.start_callback,  # Callback function
            10               # Queue size
        )

        # Creating a publisher to send detected obstacle data (angle, distance pairs) on 'lidar_val' topic
        self.publisher_ = self.create_publisher(Float32MultiArray, 'lidar_val', 10)

        # Declaring a parameter for minimum safe distance (threshold for obstacle detection)
        self.declare_parameter('obstacle_threshold', 0.5)  # meters

        # Logging node start
        self.get_logger().info("SLLIDAR Client Node Started")

        # Initializing flag to control when LiDAR scan should be processed
        self.val = True

    # Defining callback function to handle the Bool message from 'Completed' topic
    def start_callback(self, msg):
        # Enabling LiDAR scan processing once a task is completed
        self.val = True

    # Defining the callback function to process incoming LiDAR scans
    def scan_callback(self, scan):
        # Checking if scan processing is currently allowed
        if self.val:
            # Calculating number of laser readings in this scan
            count = int(scan.scan_time / scan.time_increment)

            # Retrieving the obstacle detection threshold (parameterized)
            obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value

            # Initializing the list to store obstacle information (angle and distance)
            res = []

            # Looping through each laser reading
            for i in range(count):
                # Converting the angle from radians to degrees
                degree = math.degrees(scan.angle_min + scan.angle_increment * i)

                # Checking if the detected object is closer than threshold (indicates an obstacle)
                if scan.ranges[i] < 0.5:  # Threshold hardcoded here (could use obstacle_threshold instead)
                    # Storing the angle and distance of the obstacle
                    res.append(degree)
                    res.append(scan.ranges[i])
                    # Optional debug logging:
                    # self.get_logger().info(f"Obstacle found at angle-distance: [{degree}, {scan.ranges[i]}]")

            # Creating a message with all obstacle angle-distance pairs
            msg = Float32MultiArray()
            msg.data = res

            # Logging the detected obstacle data
            self.get_logger().info(f"{res}")

            # Publishing the obstacle data to the topic 'lidar_val'
            self.publisher_.publish(msg)

            # Disabling further processing until next 'Completed' message is received
            self.val = False

# Defining the main function to run the obstacle_detection node
def main(args=None):
    # Initializing the ROS2 Python environment
    rclpy.init(args=args)

    # Creating an instance of the SLLidarClient node
    node = SLLidarClient()

    # Spinning the node to keep it active and responsive to incoming messages
    rclpy.spin(node)

    # Destroying the node after execution is stopped
    node.destroy_node()

    # Shutting down the ROS2 client library
    rclpy.shutdown()

# Executing the main function when this script is run
if __name__ == '__main__':
    main()
