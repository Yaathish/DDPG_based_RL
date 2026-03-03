import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import sys
import tty
import termios
import threading

# Key bindings
KEY_BINDINGS = {
    'w': 'forward',
    's': 'backward',
    'a': 'left',
    'd': 'right',
    'q': 'rotate_left',
    'e': 'rotate_right',
    ' ': 'stop',
}

INSTRUCTIONS = """
---------------------------
  4WD Robot Teleop Control
---------------------------
  w     : Forward
  s     : Backward
  a     : Strafe Left  (pivot left)
  d     : Strafe Right (pivot right)
  q     : Rotate Left
  e     : Rotate Right
  SPACE : Stop

  +/-   : Increase/Decrease speed
  CTRL+C: Quit
---------------------------
"""

# Turn reduction factor: inner wheels run at this fraction of full speed
# e.g. 0.4 means inner wheels run at 40% speed while outer run at 100%
# Increase toward 1.0 for wider turns, decrease toward 0.0 for sharper turns
TURN_FACTOR = 0.5

# Motor commands [M1, M2, M3, M4]
# Layout (top-down view):
#   M1(FL)  M2(FR)
#   M3(BL)  M4(BR)
# Positive = FORWARD, Negative = BACKWARD
# For left/right: outer wheels = full speed, inner wheels = reduced speed (curved turn)
MOTOR_COMMANDS = {
    'forward':      [ 1,    1,    1,    1   ],
    'backward':     [-1,   -1,   -1,   -1   ],
    'left':         [ TURN_FACTOR,  1,    TURN_FACTOR,  1   ],  # left wheels slow, right full
    'right':        [ TURN_FACTOR,    1,  1,    TURN_FACTOR],  # right wheels slow, left full
    'rotate_left':  [-1,    1,   -1,    1   ],   # still available via q/e
    'rotate_right': [ -1,   1,    1,   -1   ],
    'stop':         [ 0,    0,    0,    0   ],
}

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'mot_val', 10)
        self.speed = 150  # Default speed (0-255)
        self.get_logger().info("Teleop node started.")
        print(INSTRUCTIONS)
        print(f"Current speed: {self.speed}  (use +/- to change)\n")

    def publish_motors(self, direction):
        multipliers = MOTOR_COMMANDS[direction]
        msg = Int32MultiArray()
        msg.data = [int(m * self.speed) for m in multipliers]
        self.publisher_.publish(msg)
        if direction != 'stop':
            self.get_logger().info(f"Direction: {direction} | Values: {msg.data}")
        else:
            self.get_logger().info("STOP")

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            while rclpy.ok():
                key = get_key(settings)

                if key in KEY_BINDINGS:
                    self.publish_motors(KEY_BINDINGS[key])
                elif key == '+' or key == '=':
                    self.speed = min(255, self.speed + 10)
                    print(f"Speed: {self.speed}")
                elif key == '-':
                    self.speed = max(0, self.speed - 10)
                    print(f"Speed: {self.speed}")
                elif key == '\x03':  # CTRL+C
                    # Send stop before exiting
                    self.publish_motors('stop')
                    break

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            # Make sure robot stops on exit
            self.publish_motors('stop')


def main(args=None):
    rclpy.init(args=args)
    teleop = TeleopNode()

    # Spin in a background thread so publisher works
    spin_thread = threading.Thread(target=rclpy.spin, args=(teleop,), daemon=True)
    spin_thread.start()

    try:
        teleop.run()
    finally:
        teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()