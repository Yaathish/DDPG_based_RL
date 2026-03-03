import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math

# ─────────────────────────────────────────────
#  TUNING PARAMETERS
# ─────────────────────────────────────────────
BASE_SPEED    = 150    # Forward/backward speed (0-255)
TURN_OUTER    = 150    # Outer wheels during a turn
TURN_INNER    = 50     # Inner wheels during a turn (lower = sharper)
OBSTACLE_DIST = 0.55   # metres — obstacle trigger distance
FRONT_CONE    = 45     # degrees either side of 0° = front zone
SIDE_CONE     = 45     # degrees around ±90° = side zone
# ─────────────────────────────────────────────
# Wheel layout (top-down):
#   M1(FL)  M2(FR)
#   M3(BL)  M4(BR)
# Positive = FORWARD per your Arduino motorDir
# ─────────────────────────────────────────────


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        lidar_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, lidar_qos)

        self.motor_pub = self.create_publisher(Int32MultiArray, 'mot_val', 10)

        self.get_logger().info("Obstacle Avoidance node started.")
        self.get_logger().info(
            f"BASE_SPEED={BASE_SPEED}  OBSTACLE_DIST={OBSTACLE_DIST}m  "
            f"FRONT_CONE=±{FRONT_CONE}°  SIDE_CONE=±{SIDE_CONE}°"
        )

    def publish_motors(self, m1, m2, m3, m4, label):
        msg = Int32MultiArray()
        msg.data = [m2, m1, m3, m4]
        self.motor_pub.publish(msg)
        self.get_logger().info(f"[{label}]  motors={msg.data}")

    def angle_in_zone(self, angle_deg, center, half_width):
        """True if angle_deg is within center ± half_width, with wraparound."""
        diff = (angle_deg - center + 180) % 360 - 180
        return abs(diff) <= half_width

    def scan_callback(self, scan):
        front_dists = []
        left_dists  = []
        right_dists = []

        for i, distance in enumerate(scan.ranges):
            # Skip invalid readings
            if not math.isfinite(distance) or distance <= 0.0:
                continue

            angle_deg = math.degrees(scan.angle_min + scan.angle_increment * i)
            angle_deg = (angle_deg + 180) % 360 - 180   # normalise to [-180, 180]

            if distance < OBSTACLE_DIST:
                if self.angle_in_zone(angle_deg, 0, FRONT_CONE):
                    front_dists.append(distance)
                if self.angle_in_zone(angle_deg, 90, SIDE_CONE):
                    left_dists.append(distance)
                if self.angle_in_zone(angle_deg, -90, SIDE_CONE):
                    right_dists.append(distance)

        front_blocked = len(front_dists) > 0
        left_blocked  = len(left_dists)  > 0
        right_blocked = len(right_dists) > 0

        # ── Decision logic ──────────────────────────────────────────────
        if not front_blocked:
            # Nothing ahead — go forward
            self.publish_motors(
                BASE_SPEED, BASE_SPEED, BASE_SPEED, BASE_SPEED, "FORWARD")

        elif not right_blocked:
            # Front blocked, right is clear → turn right
            # Left wheels full, right wheels slow
            self.publish_motors(
                TURN_OUTER, TURN_INNER, TURN_OUTER, TURN_INNER, "TURN-RIGHT")

        elif not left_blocked:
            # Front blocked, right blocked, left clear → turn left
            # Right wheels full, left wheels slow
            self.publish_motors(
                TURN_INNER, TURN_OUTER, TURN_INNER, TURN_OUTER, "TURN-LEFT")

        else:
            # Everything blocked → reverse
            self.publish_motors(
                -BASE_SPEED, -BASE_SPEED, -BASE_SPEED, -BASE_SPEED, "REVERSE")


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    stop_msg = Int32MultiArray()
    stop_msg.data = [0, 0, 0, 0]
    node.motor_pub.publish(stop_msg)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()