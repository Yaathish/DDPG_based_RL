# Importing ROS2 Python client library for creating nodes and interfacing with ROS2
import rclpy
# Importing the Node base class from ROS2 to define custom nodes
from rclpy.node import Node
# Importing standard ROS2 message types for exchanging arrays of float and integer values
from std_msgs.msg import Float32MultiArray, Int32MultiArray
# Importing GPIOZero's Button class to interface with the encoder hardware connected to GPIO pins
from gpiozero import Button
# Importing time module for tracking timestamps of encoder pulses
import time
# Importing math module to use mathematical constants and formulas (e.g., π)
import math

# Defining the Encoder_publish class which inherits from the ROS2 Node class
class Encoder_publish(Node):
    # Initializing the encoder_pub node
    def __init__(self):
        super().__init__('encoder_pub')  # Naming the node 'encoder_pub'

        # Creating a ROS2 publisher that publishes velocity data as Float32MultiArray on topic 'enc_val'
        self.publisher_ = self.create_publisher(Float32MultiArray, 'enc_val', 10)

        # Creating a subscriber that listens to motor direction signs from topic 'mot_sign'
        self.subscription = self.create_subscription(
            Int32MultiArray,           # Message type
            'mot_sign',                # Topic name
            self.listener_callback,    # Callback function when message is received
            10                         # Queue size
        )

        # Initializing motor direction sign list for 4 motors (1 or -1)
        self.sign = [1, 1, 1, 1]

        # Defining timer period (30ms) to compute and publish velocity at regular intervals
        self.timer_period = 0.03
        # Creating a ROS2 timer to call timer_callback every 30ms
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Initializing pulse counts for all 4 encoders
        self.count = [0, 0, 0, 0]

        # Initializing GPIO pins connected to the encoder outputs
        self.encoder = [Button(14), Button(27), Button(24), Button(25)]

        # Defining the wheel diameter in meters
        self.diameter = 0.065  # meters

        # Defining Pulses Per Revolution (PPR) of the encoder
        self.ppr = 20  # pulses per revolution

        # Initializing last pulse time for each encoder to handle timeout-based zeroing
        self.prev = [0, 0, 0, 0]

        # Attaching encoder interrupt callbacks using GPIOZero's 'when_pressed' for pulse counting
        for i in range(4):
            self.encoder[i].when_pressed = lambda i=i: self.enco_call(i)

    # Defining the callback function for motor sign subscription
    def listener_callback(self, msg):
        val = msg.data  # Extracting list of motor directions from incoming message

        # Validating that 4 direction values are received
        if len(msg.data) != 4:
            self.get_logger().error(f"Received incomplete data: {msg.data}")
            return

        # Updating the sign list: +1 for forward, -1 for reverse
        for i in range(4):
            self.sign[i] = -1 if val[i] < 0 else 1

    # Defining the timer callback to compute and publish velocity
    def timer_callback(self):
        # Computing wheel velocities using: velocity = (sign × π × diameter × pulses) / (PPR × time interval)
        velocity = [(self.sign[i] * math.pi * self.diameter * self.count[i]) / (self.ppr * self.timer_period) for i in range(4)]

        # Resetting the pulse count after velocity computation
        self.count = [0, 0, 0, 0]

        # Setting velocity to 0 if no pulse was received in last 1 second (motor stalled)
        for i in range(4):
            if time.time() - self.prev[i] > 1:
                velocity[i] = 0

        # Creating and publishing velocity message
        msg = Float32MultiArray()
        msg.data = velocity
        self.publisher_.publish(msg)

        # Logging the published velocity values
        self.get_logger().info('Publishing: "%s"' % msg.data)

    # Defining the encoder callback function to increment pulse count when a pulse is detected
    def enco_call(self, no):
        # Incrementing the count for the encoder that triggered the interrupt
        self.count[no] += 1

        # Updating the last pulse timestamp for that encoder
        self.prev[no] = time.time()

# Defining the main function to start the ROS2 node
def main(args=None):
    # Initializing ROS2 communication
    rclpy.init(args=args)
    
    # Creating an instance of the Encoder_publish node
    encoder_publish = Encoder_publish()

    # Spinning the node so it keeps running and handling callbacks
    rclpy.spin(encoder_publish)

    # Destroying the node instance on shutdown
    encoder_publish.destroy_node()

    # Shutting down the ROS2 communication
    rclpy.shutdown()

# Executing the main function when this file is run as a script
if __name__ == '__main__':
    main()
