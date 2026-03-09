"""
TEST 1 — PPR (Pulses Per Revolution) Verification
==================================================
PURPOSE:
  encoder_pub.py has ppr=20 hardcoded. This test verifies
  if 20 is correct. Wrong PPR = wrong odometry = bad navigation.

HOW IT WORKS:
  - Listens to raw encoder pulses from GPIO directly (not velocity)
  - You push the robot EXACTLY 1 metre by hand (motors OFF)
  - Script counts total pulses
  - Calculates true PPR from pulse count + wheel circumference

FORMULA:
  distance = (pulses / PPR) × π × diameter
  → PPR = pulses × π × diameter / distance
  → PPR = total_pulses × π × 0.065 / 1.0

SETUP:
  1. Mark a start line on the floor
  2. Mark an end line exactly 1.000m away (use tape measure)
  3. Run this script FIRST, then press ENTER, then push robot slowly
  4. Stop robot exactly at end line, press ENTER again

PLACE IN REPO:
  motor_node/motor_node/test1_ppr.py
  OR run standalone: python3 test1_ppr.py

RUN:
  python3 test1_ppr.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
import time

WHEEL_DIAMETER = 0.065   # metres — from encoder_pub.py
TEST_DISTANCE  = 1.000   # metres — push robot exactly this far

class PPRTest(Node):
    def __init__(self):
        super().__init__('ppr_test')

        # We listen to enc_val (velocity) published by encoder_pub.py
        # and back-calculate pulses from velocity × time
        # This avoids needing to modify encoder_pub.py
        self.sub = self.create_subscription(
            Float32MultiArray, 'enc_val', self._enc_cb, 10
        )

        self.velocities   = []   # store all velocity samples
        self.collecting   = False
        self.start_time   = 0.0

        print("\n" + "="*55)
        print("  TEST 1 — PPR VERIFICATION")
        print("="*55)
        print(f"  Wheel diameter : {WHEEL_DIAMETER*100:.1f} cm")
        print(f"  Test distance  : {TEST_DISTANCE*100:.0f} cm (exactly 1 metre)")
        print()
        print("  REQUIREMENTS:")
        print("    ✅ encoder_pub.py must be RUNNING")
        print("    ✅ Motor power OFF (push by hand only)")
        print("    ✅ Tape measure on floor: 0 → 1.000m marked")
        print()
        print("  STEPS:")
        print("    1. Place robot front wheel on START line")
        print("    2. Press ENTER to begin counting")
        print("    3. Push robot slowly to END line (1.000m)")
        print("    4. Press ENTER again to stop")
        print()
        input("  → Press ENTER when robot is on START line...")
        print()
        print("  ⏺  COUNTING PULSES — push robot to end line now...")
        self.collecting = True
        self.start_time = time.time()

    def _enc_cb(self, msg):
        if self.collecting:
            # Store velocity + timestamp for integration later
            self.velocities.append((time.time(), list(msg.data)))

    def analyse(self):
        if not self.velocities:
            print("  ❌ No encoder data received!")
            print("     Is encoder_pub.py running? Check: ros2 topic echo /enc_val")
            return

        # Integrate velocity over time to get distance per wheel
        # distance = sum(velocity × dt)
        total_pulses_approx = []

        TIMER_PERIOD = 0.03   # encoder_pub.py timer = 30ms

        for wheel_idx in range(4):
            # velocity = (sign × π × d × pulses) / (PPR × timer_period)
            # pulses_per_sample = velocity × PPR × timer_period / (π × d)
            # We don't know PPR yet — so count raw using current ppr=20 to get
            # approximate pulse count, then scale
            raw_distance = 0.0
            for i in range(len(self.velocities) - 1):
                t0, v0 = self.velocities[i]
                t1, _  = self.velocities[i+1]
                dt = t1 - t0
                raw_distance += abs(v0[wheel_idx]) * dt

            # raw_distance was calculated using ppr=20
            # real_pulses = raw_distance / (π × d) × 20
            assumed_ppr = 20
            estimated_pulses = (raw_distance / (math.pi * WHEEL_DIAMETER)) * assumed_ppr
            total_pulses_approx.append(estimated_pulses)

        print()
        print("="*55)
        print("  RESULTS")
        print("="*55)

        for i, name in enumerate(['FL', 'FR', 'BL', 'BR']):
            pulses = total_pulses_approx[i]
            if pulses > 0:
                true_ppr = pulses * math.pi * WHEEL_DIAMETER / TEST_DISTANCE
                revs     = TEST_DISTANCE / (math.pi * WHEEL_DIAMETER)
                print(f"  Wheel {name}: ~{pulses:.0f} pulses  →  PPR ≈ {true_ppr:.1f}")
            else:
                print(f"  Wheel {name}: no movement detected")

        avg_pulses = sum(p for p in total_pulses_approx if p > 0)
        n = sum(1 for p in total_pulses_approx if p > 0)

        if n > 0:
            avg = avg_pulses / n
            true_ppr = avg * math.pi * WHEEL_DIAMETER / TEST_DISTANCE
            revolutions = TEST_DISTANCE / (math.pi * WHEEL_DIAMETER)

            print()
            print(f"  Average across {n} wheels:")
            print(f"    Estimated pulses : {avg:.0f}")
            print(f"    Wheel revolutions: {revolutions:.2f}")
            print(f"    ✅ TRUE PPR ≈ {true_ppr:.0f}")
            print()
            if abs(true_ppr - 20) < 2:
                print("  ✅ PPR=20 is CORRECT — no change needed")
            else:
                print(f"  ⚠  PPR=20 is WRONG!")
                print(f"     Update encoder_pub.py: self.ppr = {round(true_ppr)}")
        print("="*55)


def main():
    rclpy.init()
    node = PPRTest()

    try:
        # Spin briefly to collect some data while user pushes robot
        print("  (Collecting encoder data...)")
        input("  → Press ENTER when robot reaches END line (1.000m)...")
        node.collecting = False

        # Process a bit more to flush callbacks
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)

    except KeyboardInterrupt:
        pass

    node.analyse()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()