"""
TEST 2 — Maximum Linear Velocity
==================================
PURPOSE:
  td3_inference.py sends cmd_vel with max linear = 0.30 m/s.
  This test confirms whether the real robot CAN do 0.30 m/s
  and what the actual maximum is at PWM=255.

  If real max < 0.30 → robot never reaches full speed → bad navigation
  If real max > 0.30 → robot is slower than simulation → acceptable

HOW IT WORKS:
  - Sends PWM=255 to all 4 wheels for 3 seconds
  - Reads /enc_val velocity during that time
  - Calculates average steady-state velocity

SETUP:
  1. Clear 3+ metres of floor space ahead of robot
  2. Motor power ON
  3. Hold robot or put on blocks until script says GO

PLACE IN REPO:
  motor_node/motor_node/test2_max_linear_vel.py

RUN:
  Terminal 1: ros2 run encoder_node encoder_pub
  Terminal 2: ros2 run motor_node motor_sub
  Terminal 3: python3 test2_max_linear_vel.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import time
import math

TEST_DURATION = 3.0    # seconds of full speed
WARMUP_TIME   = 0.5    # seconds to ignore (motor spin-up)
PWM_FULL      = 255

class MaxLinearVelTest(Node):
    def __init__(self):
        super().__init__('test2_max_linear_vel')

        self.motor_pub = self.create_publisher(Int32MultiArray, 'mot_val', 10)
        self.enc_sub   = self.create_subscription(
            Float32MultiArray, 'enc_val', self._enc_cb, 10
        )

        self.velocities  = []
        self.test_active = False
        self.start_time  = 0.0

        print("\n" + "="*55)
        print("  TEST 2 — MAXIMUM LINEAR VELOCITY")
        print("="*55)
        print(f"  PWM setting  : {PWM_FULL} (full power)")
        print(f"  Test duration: {TEST_DURATION:.0f} seconds")
        print()
        print("  REQUIREMENTS:")
        print("    ✅ encoder_pub running (ros2 run encoder_node encoder_pub)")
        print("    ✅ motor_sub running   (ros2 run motor_node motor_sub)")
        print("    ✅ 3+ metres clear space ahead of robot")
        print("    ✅ Battery fully charged")
        print()
        print("  ⚠  Robot will drive FORWARD at FULL SPEED for 3 seconds!")
        print("  ⚠  Make sure path is clear!")
        print()
        input("  → Press ENTER to start (robot will move in 3 seconds)...")

        print()
        for i in range(3, 0, -1):
            print(f"  Starting in {i}...")
            time.sleep(1.0)

    def _enc_cb(self, msg):
        if self.test_active:
            elapsed = time.time() - self.start_time
            if elapsed > WARMUP_TIME:   # skip spin-up phase
                # Average of all 4 wheel velocities (all should be positive)
                avg_vel = sum(abs(v) for v in msg.data) / 4.0
                self.velocities.append((elapsed, avg_vel, list(msg.data)))

    def _stop(self):
        msg = Int32MultiArray()
        msg.data = [0, 0, 0, 0]
        for _ in range(5):
            self.motor_pub.publish(msg)
            time.sleep(0.05)
        print("  🛑 Motors stopped")

    def run(self):
        # Start motors at full speed
        cmd = Int32MultiArray()
        cmd.data = [PWM_FULL, PWM_FULL, PWM_FULL, PWM_FULL]
        self.motor_pub.publish(cmd)
        self.test_active = True
        self.start_time  = time.time()
        print(f"  🚀 Running at PWM={PWM_FULL}...")

        deadline = self.start_time + TEST_DURATION
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        self._stop()
        self.test_active = False

    def analyse(self):
        if not self.velocities:
            print("  ❌ No velocity data! Check encoder_pub is running.")
            return

        # Steady state = last 60% of test (skip spin-up)
        n = len(self.velocities)
        steady = self.velocities[int(n * 0.4):]

        avg_vel = sum(v[1] for v in steady) / len(steady)
        max_vel = max(v[1] for v in steady)
        min_vel = min(v[1] for v in steady)

        print()
        print("="*55)
        print("  RESULTS — MAX LINEAR VELOCITY")
        print("="*55)
        print(f"  Samples collected : {len(self.velocities)}")
        print(f"  Steady-state avg  : {avg_vel:.3f} m/s")
        print(f"  Peak velocity     : {max_vel:.3f} m/s")
        print(f"  Min velocity      : {min_vel:.3f} m/s")
        print()

        # Per-wheel breakdown
        print("  Per-wheel breakdown (steady state avg):")
        for i, name in enumerate(['FL', 'FR', 'BL', 'BR']):
            wheel_avg = sum(abs(v[2][i]) for v in steady) / len(steady)
            print(f"    {name}: {wheel_avg:.3f} m/s")

        print()
        print(f"  Simulation uses  : 0.300 m/s max")
        print(f"  Real robot max   : {avg_vel:.3f} m/s")

        if avg_vel >= 0.28:
            print(f"  ✅ GOOD — real robot matches simulation (≥0.28 m/s)")
            print(f"     Keep td3_env.py: lin_fwd × 0.30 m/s")
        elif avg_vel >= 0.20:
            print(f"  ⚠  SLOWER than simulation")
            print(f"     Update td3_inference.py MAX_LINEAR = {avg_vel:.2f}")
        else:
            print(f"  ❌ Very slow — check battery, motor connections")

        print("="*55)


def main():
    rclpy.init()
    node = MaxLinearVelTest()

    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.analyse()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()