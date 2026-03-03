# Importing ROS2 Python client library to create nodes and manage communication
import rclpy
# Importing Node class from ROS2 for node creation
from rclpy.node import Node
# Importing message type used to receive and publish motor velocity data
from std_msgs.msg import Int32MultiArray
# Importing serial library for communication with external microcontroller (Arduino)
import serial

# Defining the MotorSubscriber node class responsible for sending motor values to Arduino
class MotorSubscriber(Node):
    # Initializing the node
    def __init__(self):
        # Initializing the ROS2 node with the name 'motor_sub'
        super().__init__('motor_sub')

        # Creating a subscription to the 'mot_val' topic, which carries desired motor speeds
        self.subscription = self.create_subscription(
            Int32MultiArray,           # Expected message type: array of 4 integers (one per motor)
            'mot_val',                 # Topic name to subscribe to
            self.listener_callback,    # Callback function to be called on receiving data
            10                         # Queue size
        )
        # Referencing the subscription object to avoid linter warnings about unused variables
        self.subscription

        # Creating a publisher to publish the sign (+1 or -1) of each motor command on the 'mot_sign' topic
        self.publisher_ = self.create_publisher(Int32MultiArray, 'mot_sign', 10)

        # Attempting to open the serial port to communicate with the Arduino
        try:
            # Opening the serial port '/dev/ttyUSB1' with baudrate 115200 and timeout 1 second
            self.arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            # Logging success message if the port opens correctly
            self.get_logger().info("Serial port opened successfully.")
        except serial.SerialException as e:
            # Logging error if the serial port fails to open
            self.get_logger().error(f"Could not open serial port: {e}")

    # Defining the callback function that is triggered when new motor values are received
    def listener_callback(self, msg):
        # Storing the received motor command list (4 motor values)
        val = msg.data

        # Verifying that all 4 motor values are present
        if len(msg.data) != 4:
            self.get_logger().error(f"Received incomplete data: {msg.data}")
            return

        # Formatting the command as a string to send to Arduino via serial, e.g., "100,-50,75,-100\n"
        command = f"{val[0]},{val[1]},{val[2]},{val[3]}\n"

        # Sending the command over the serial port to Arduino
        self.arduino.write(command.encode('utf-8'))

        # Logging the command sent for debugging
        self.get_logger().info(f"Sent: {command.strip()}")

        # Creating a new message to publish the sign of each motor speed
        msg1 = Int32MultiArray()
        # Calculating sign (+1 or -1) for each motor speed; if 0, defaulting to +1
        msg1.data = [int(val[i]/abs(val[i])) if val[i] else 1 for i in range(4)]

        # Publishing the sign array to the 'mot_sign' topic
        self.publisher_.publish(msg1)

# Defining the main function to start the MotorSubscriber node
def main(args=None):
    # Initializing the ROS2 Python environment
    rclpy.init(args=args)

    # Creating an instance of the MotorSubscriber node
    motor_subscriber = MotorSubscriber()

    # Spinning the node to keep it alive and responding to messages
    rclpy.spin(motor_subscriber)

    # Destroying the node once execution is complete
    motor_subscriber.destroy_node()

    # Shutting down the ROS2 system
    rclpy.shutdown()

# Ensuring the script is run directly (not imported as a module)
if __name__ == '__main__':
    main()