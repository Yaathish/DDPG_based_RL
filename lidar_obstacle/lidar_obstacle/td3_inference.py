"""
td3_inference.py — Pi 5 TD3 Inference Node (ROS2 Jazzy)
=========================================================
Uses sensor_fusion.py for odometry instead of raw encoders.
This gives much better position accuracy.

Run order:
  Terminal 1: ros2 launch sllidar_ros2 sllidar_a1_launch.py
  Terminal 2: ros2 run <motor_pkg> motor_pub
  Terminal 3: ros2 run lidar_obstacle encoder_pub
  Terminal 4: ros2 run lidar_obstacle sensor_fusion
  Terminal 5: ros2 run lidar_obstacle td3_inference

Topics:
  SUB  /scan    LaserScan     ← RPLiDAR A1
  SUB  /odom    Odometry      ← from sensor_fusion node
  PUB  /mot_val Int32MultiArray → motors [FL, FR, RL, RR]
  PUB  /mot_sign Int32MultiArray → motor direction signs for encoder_pub
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math, time, os
import numpy as np
import torch
from actor_critic import Actor, STATE_DIM, ACTION_DIM

# ── MUST MATCH TRAINING VALUES EXACTLY ────────────────────────────────
NUM_LIDAR_BINS = 24
MAX_LIDAR_DIST = 3.5
GOAL_TOLERANCE = 0.20

# ── Motor scaling ──────────────────────────────────────────────────────
MAX_LIN_MOTOR = 200
MAX_ANG_MOTOR = 150

# ── Model path ────────────────────────────────────────────────────────
MODEL_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "TD3_actor.pth"
)


class TD3Inference(Node):
    def __init__(self):
        super().__init__("td3_inference")

        # ── Load model ─────────────────────────────────────────────────
        if not os.path.exists(MODEL_PATH):
            self.get_logger().error(f"❌ TD3_actor.pth not found at {MODEL_PATH}")
            raise FileNotFoundError(MODEL_PATH)

        self.actor = Actor(state_dim=STATE_DIM, action_dim=ACTION_DIM)
        self.actor.load_state_dict(torch.load(MODEL_PATH, map_location="cpu"))
        self.actor.eval()
        self.get_logger().info(f"✅ Model loaded: {MODEL_PATH}")

        # ── Goal position ──────────────────────────────────────────────
        # Set this to your real-world goal (metres from robot start)
        self._goal_x = 0.75
        self._goal_y = 0.00
        self.get_logger().info(
            f"🎯 Goal: ({self._goal_x}, {self._goal_y})  "
            f"dist={math.hypot(self._goal_x, self._goal_y):.2f}m"
        )

        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)

        # Subscribers
        self.scan_sub  = self.create_subscription(LaserScan, "/scan", self._scan_cb, qos)
        self.odom_sub  = self.create_subscription(Odometry,  "/odom", self._odom_cb, 10)

        # Publishers
        self.motor_pub = self.create_publisher(Int32MultiArray, "/mot_val",  10)
        self.sign_pub  = self.create_publisher(Int32MultiArray, "/mot_sign", 10)

        # ── State ──────────────────────────────────────────────────────
        self._lidar_bins = [MAX_LIDAR_DIST] * NUM_LIDAR_BINS
        self._x          = 0.0
        self._y          = 0.0
        self._theta      = 0.0

        self._arrived    = False
        self._odom_ready = False

        # 10 Hz control loop
        self.create_timer(0.1, self._loop)

        self.get_logger().info("TD3 Inference ready — waiting for sensors...")

    # ── Callbacks ──────────────────────────────────────────────────────
    def _scan_cb(self, msg):
        bins      = [MAX_LIDAR_DIST] * NUM_LIDAR_BINS
        bin_width = 360.0 / NUM_LIDAR_BINS
        for i, d in enumerate(msg.ranges):
            if not math.isfinite(d) or d < 0.15:
                continue
            a   = math.degrees(msg.angle_min + i * msg.angle_increment) % 360
            idx = int(a / bin_width) % NUM_LIDAR_BINS
            bins[idx] = min(bins[idx], d)
        self._lidar_bins = bins

    def _odom_cb(self, msg):
        """Use fused odometry from sensor_fusion node."""
        self._x     = msg.pose.pose.position.x
        self._y     = msg.pose.pose.position.y
        q           = msg.pose.pose.orientation
        self._theta = math.atan2(
            2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z)
        )
        self._odom_ready = True

    # ── Build observation — MUST match training exactly ────────────────
    def _build_obs(self):
        lidar_norm = [min(b, MAX_LIDAR_DIST) / MAX_LIDAR_DIST for b in self._lidar_bins]
        dist       = math.hypot(self._goal_x - self._x, self._goal_y - self._y)
        abs_a      = math.atan2(self._goal_y - self._y, self._goal_x - self._x)
        rel_a      = (abs_a - self._theta + math.pi) % (2*math.pi) - math.pi
        dist_norm  = min(dist, 3.0) / 3.0
        angle_norm = rel_a / math.pi
        return np.array(lidar_norm + [dist_norm, angle_norm], dtype=np.float32)

    # ── Control loop ───────────────────────────────────────────────────
    def _loop(self):
        if self._arrived:
            return

        if not self._odom_ready:
            self.get_logger().info("Waiting for /odom from sensor_fusion...", once=True)
            return

        dist = math.hypot(self._goal_x - self._x, self._goal_y - self._y)

        if dist < GOAL_TOLERANCE:
            self._stop()
            self._arrived = True
            self.get_logger().info(
                f"🏁 GOAL REACHED! pos=({self._x:.2f},{self._y:.2f}) dist={dist:.3f}m"
            )
            return

        obs = self._build_obs()
        with torch.no_grad():
            action = self.actor(
                torch.FloatTensor(obs).unsqueeze(0)
            ).squeeze(0).numpy()

        lin_vel = float(action[0])
        ang_vel = float(action[1])

        # Match training: lin [-1,1] → [0,1] forward only
        lin_fwd = (lin_vel + 1.0) / 2.0
        base    = int(lin_fwd * MAX_LIN_MOTOR)
        diff    = int(ang_vel  * MAX_ANG_MOTOR)

        fl = max(-250, min(250, base - diff))
        fr = max(-250, min(250, base + diff))
        rl = max(-250, min(250, base - diff))
        rr = max(-250, min(250, base + diff))

        # ── Physical motor layout (from robot wiring diagram) ──────────
        #   Index: [M1,   M2,   M3,   M4  ]
        #   Pos:   [BR,   BL,   FL,   FR  ]
        # So we map:
        #   M1 (index 0) = Back-Right  = rr
        #   M2 (index 1) = Back-Left   = rl
        #   M3 (index 2) = Front-Left  = fl
        #   M4 (index 3) = Front-Right = fr
        mot_msg = Int32MultiArray()
        mot_msg.data = [rr, rl, fl, fr]   # [M1, M2, M3, M4]
        self.motor_pub.publish(mot_msg)

        # Publish motor signs for encoder_pub direction tracking
        sign_msg = Int32MultiArray()
        sign_msg.data = [
            1 if rr >= 0 else -1,   # M1 = BR
            1 if rl >= 0 else -1,   # M2 = BL
            1 if fl >= 0 else -1,   # M3 = FL
            1 if fr >= 0 else -1,   # M4 = FR
        ]
        self.sign_pub.publish(sign_msg)

        self.get_logger().info(
            f"pos=({self._x:.2f},{self._y:.2f}) "
            f"hdg={math.degrees(self._theta):+.0f}° "
            f"dist={dist:.2f}m | "
            f"lin={lin_vel:+.2f} ang={ang_vel:+.2f} | "
            f"FL={fl:+4d} FR={fr:+4d}"
        )

    def _stop(self):
        msg = Int32MultiArray()
        msg.data = [0, 0, 0, 0]
        self.motor_pub.publish(msg)
        sign_msg = Int32MultiArray()
        sign_msg.data = [1, 1, 1, 1]
        self.sign_pub.publish(sign_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TD3Inference()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node._stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()