"""
sensor_fusion.py — LiDAR + Encoder Odometry Fusion for Pi 5
=============================================================
Publishes a fused /odom topic that td3_inference.py uses.

Why fusion?
  - Encoder odometry alone drifts (wheel slip, uneven floor)
  - LiDAR can detect walls to correct position
  - Fusion = more accurate robot position = better navigation

Topics:
  SUB  /scan     LaserScan           ← RPLiDAR A1
  SUB  /enc_val  Float32MultiArray   ← wheel velocities from encoder_pub
  PUB  /odom     Odometry            ← fused position estimate
  PUB  /odom_path Path               ← trail for RViz visualization

Place in: ~/project/src/lidar_obstacle/lidar_obstacle/sensor_fusion.py

Add to setup.py entry_points:
  'sensor_fusion = lidar_obstacle.sensor_fusion:main'

Run: ros2 run lidar_obstacle sensor_fusion
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math, numpy as np, time

# ── Robot physical parameters ──────────────────────────────────────────
WHEELBASE      = 0.155   # metres (from URDF)
WHEEL_DIAMETER = 0.065   # metres
PPR            = 20      # ← UPDATE after running ppr_test.py!

# ── Arena walls (for wall-based correction) ────────────────────────────
ARENA_SIZE = 1.0   # walls at ±1.0m from centre

# ── LiDAR wall correction settings ────────────────────────────────────
WALL_CORRECTION_ENABLED = True
WALL_DIST_THRESHOLD     = 0.60   # only correct if within 0.60m of a wall
CORRECTION_GAIN         = 0.15   # how strongly to pull toward wall estimate


class SensorFusion(Node):
    def __init__(self):
        super().__init__("sensor_fusion")

        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)

        # Subscribers
        self.enc_sub  = self.create_subscription(
            Float32MultiArray, "/enc_val", self._enc_cb, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self._scan_cb, qos)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "/odom",      10)
        self.path_pub = self.create_publisher(Path,     "/odom_path", 10)

        # TF broadcaster (so RViz can show robot position)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── State ──────────────────────────────────────────────────────
        self._x      = 0.0
        self._y      = 0.0
        self._theta  = 0.0   # radians

        # Encoder velocities [FL, FR, RL, RR] in m/s
        self._wheel_vel = [0.0, 0.0, 0.0, 0.0]

        # Last LiDAR scan ranges
        self._scan_ranges    = []
        self._scan_angle_min = 0.0
        self._scan_angle_inc = 0.0

        self._last_t = time.time()

        # Path for visualization
        self._path = Path()
        self._path.header.frame_id = "odom"

        # 10 Hz update loop
        self.create_timer(0.1, self._update)

        self.get_logger().info("Sensor Fusion node ready ✅")
        self.get_logger().info(f"  PPR={PPR}  Wheelbase={WHEELBASE}m")
        self.get_logger().info("  Subscribing to /enc_val and /scan")

    # ── Callbacks ──────────────────────────────────────────────────────
    def _enc_cb(self, msg):
        if len(msg.data) == 4:
            self._wheel_vel = list(msg.data)

    def _scan_cb(self, msg):
        self._scan_ranges    = list(msg.ranges)
        self._scan_angle_min = msg.angle_min
        self._scan_angle_inc = msg.angle_increment

    # ── Encoder odometry (skid steer) ──────────────────────────────────
    def _encoder_odom(self, dt):
        """
        Skid steer odometry from wheel velocities.
        Left wheels:  FL(0) + RL(3)
        Right wheels: FR(1) + RR(2)
        """
        v_left  = (self._wheel_vel[0] + self._wheel_vel[3]) / 2.0
        v_right = (self._wheel_vel[1] + self._wheel_vel[2]) / 2.0

        v     = (v_right + v_left)  / 2.0
        omega = (v_right - v_left)  / WHEELBASE

        dx = v * math.cos(self._theta) * dt
        dy = v * math.sin(self._theta) * dt
        dtheta = omega * dt

        return dx, dy, dtheta, v, omega

    # ── LiDAR wall correction ──────────────────────────────────────────
    def _wall_correction(self):
        """
        Use LiDAR readings pointing at walls to correct x,y drift.
        Only corrects when robot is close enough to a wall for reliable reading.

        Strategy:
          - Find minimum range in cardinal directions (front/back/left/right)
          - If min range < threshold, infer wall position and correct estimate
        """
        if not self._scan_ranges or not WALL_CORRECTION_ENABLED:
            return 0.0, 0.0

        ranges = self._scan_ranges
        n      = len(ranges)
        if n == 0:
            return 0.0, 0.0

        def get_range_at_angle(angle_deg):
            """Get LiDAR range at specific angle relative to robot."""
            angle_rad = math.radians(angle_deg)
            idx = int((angle_rad - self._scan_angle_min) / self._scan_angle_inc)
            idx = max(0, min(idx, n - 1))
            d = ranges[idx]
            return d if math.isfinite(d) and 0.12 < d < 5.0 else None

        dx_corr = 0.0
        dy_corr = 0.0

        # ── Front wall (robot heading direction) ──────────────────────
        d_front = get_range_at_angle(0)
        if d_front and d_front < WALL_DIST_THRESHOLD:
            # Expected x if robot is d_front from north wall
            expected_x = ARENA_SIZE - d_front
            # Current heading projection
            wx = self._x + d_front * math.cos(self._theta)
            if abs(wx) > 0.7:   # near east or west wall
                dx_corr = CORRECTION_GAIN * (
                    math.copysign(ARENA_SIZE, wx) - d_front * math.cos(self._theta) - self._x
                )

        # ── Right wall ─────────────────────────────────────────────────
        d_right = get_range_at_angle(-90)
        if d_right and d_right < WALL_DIST_THRESHOLD:
            wy = self._y - d_right * math.sin(self._theta + math.pi/2)
            if abs(wy) > 0.7:
                dy_corr = CORRECTION_GAIN * (
                    math.copysign(ARENA_SIZE, wy) + d_right - self._y
                )

        return dx_corr, dy_corr

    # ── Main update loop ───────────────────────────────────────────────
    def _update(self):
        now = time.time()
        dt  = min(now - self._last_t, 0.2)
        self._last_t = now

        # 1. Encoder odometry
        dx, dy, dtheta, v, omega = self._encoder_odom(dt)

        # 2. Update position
        self._x     += dx
        self._y     += dy
        self._theta += dtheta
        self._theta  = (self._theta + math.pi) % (2 * math.pi) - math.pi

        # 3. LiDAR wall correction (reduces drift)
        dx_c, dy_c   = self._wall_correction()
        self._x     += dx_c
        self._y     += dy_c

        # 4. Clamp to arena bounds (robot can't be outside walls)
        self._x = max(-ARENA_SIZE + 0.12, min(ARENA_SIZE - 0.12, self._x))
        self._y = max(-ARENA_SIZE + 0.12, min(ARENA_SIZE - 0.12, self._y))

        # 5. Publish odometry
        self._publish_odom(v, omega)
        self._publish_tf()
        self._update_path()

    def _publish_odom(self, v, omega):
        msg = Odometry()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id  = "base_footprint"

        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.position.z = 0.0

        # Convert heading to quaternion
        msg.pose.pose.orientation.z = math.sin(self._theta / 2)
        msg.pose.pose.orientation.w = math.cos(self._theta / 2)

        msg.twist.twist.linear.x  = v
        msg.twist.twist.angular.z = omega

        self.odom_pub.publish(msg)

    def _publish_tf(self):
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id  = "base_footprint"

        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = 0.0
        t.transform.rotation.z    = math.sin(self._theta / 2)
        t.transform.rotation.w    = math.cos(self._theta / 2)

        self.tf_broadcaster.sendTransform(t)

    def _update_path(self):
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = "odom"
        pose.pose.position.x = self._x
        pose.pose.position.y = self._y
        pose.pose.orientation.z = math.sin(self._theta / 2)
        pose.pose.orientation.w = math.cos(self._theta / 2)

        self._path.poses.append(pose)
        if len(self._path.poses) > 1000:
            self._path.poses.pop(0)

        self._path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self._path)

    def reset_pose(self):
        """Call this to reset odometry to (0,0,0) at start of each run."""
        self._x     = 0.0
        self._y     = 0.0
        self._theta = 0.0
        self._path.poses.clear()
        self.get_logger().info("Odometry reset to (0, 0, 0°)")


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()