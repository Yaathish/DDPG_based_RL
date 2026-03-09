"""
TEST 3 — Maximum Angular Velocity
====================================
PURPOSE:
  td3_inference.py maps action[1] → ±1.50 rad/s angular velocity.
  This test measures real angular velocity at full PWM.

  1.50 rad/s = one full rotation in ~4.2 seconds.
  If real max differs, update td3_inference.py.

HOW IT WORKS:
  - Spins robot in place (left wheels backward, right forward)
  - Times exactly ONE full 360° rotation
  - Calculates angular velocity = 2π / time_for_360

  ALSO calculates from encoder:
  - angular_vel = (v_right - v_left) / wheelbase
  - wheelbase assumed 0.155m — update if you measured differently

SETUP:
  1. Put robot on smooth floor with room to spin
  2. Put a piece of tape on top pointing forward as reference
  3. Motor power ON

PLACE IN REPO:
  motor_node/motor_node/test3_max_angular_vel.py

RUN:
  Terminal 1: ros2 run encoder_node encoder_pub
  Terminal 2: ros2 run motor_node motor_sub
  Terminal 3: python3 test3_max_angular_vel.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import time
import math

WHEELBASE     = 0.155   # metres — update after measuring with ruler
PWM_FULL      = 200     # use 200 not 255 — safer for rotation test
SPIN_DURATION = 5.0     # seconds — enough for 1 full rotation

class MaxAngularVelTest(Node):
    def __init__(self):
        super().__init__('test3_max_angular_vel')

        self.motor_pub = self.create_publisher(Int32MultiArray, 'mot_val', 10)
        self.enc_sub   = self.create_subscription(
            Float32MultiArray, 'enc_val', self._enc_cb, 10
        )

        self.velocities  = []
        self.test_active = False
        self.start_time  = 0.0

        print("\n" + "="*55)
        print("  TEST 3 — MAXIMUM ANGULAR VELOCITY")
        print("="*55)
        print(f"  PWM setting  : {PWM_FULL}")
        print(f"  Wheelbase    : {WHEELBASE*100:.1f} cm (update if wrong)")
        print(f"  Spin duration: {SPIN_DURATION:.0f} seconds")
        print()
        print("  METHOD A (automatic): encoder-based calculation")
        print("  METHOD B (manual)   : time one full 360° rotation")
        print()
        print("  REQUIREMENTS:")
        print("    ✅ encoder_pub running")
        print("    ✅ motor_sub running")
        print("    ✅ Robot on smooth floor")
        print("    ✅ Reference mark on top of robot (tape)")
        print()
        print("  ⚠  Robot will SPIN IN PLACE for 5 seconds!")
        print()
        input("  → Press ENTER to start spinning...")

        print()
        for i in range(3, 0, -1):
            print(f"  Starting in {i}...")
            time.sleep(1.0)

    def _enc_cb(self, msg):
        if self.test_active:
            elapsed = time.time() - self.start_time
            if elapsed > 0.3:   # skip first 0.3s spin-up
                self.velocities.append((elapsed, list(msg.data)))

    def _stop(self):
        msg = Int32MultiArray()
        msg.data = [0, 0, 0, 0]
        for _ in range(5):
            self.motor_pub.publish(msg)
            time.sleep(0.05)
        print("  🛑 Motors stopped")

    def run(self):
        # Rotate left: left wheels backward, right wheels forward
        # M1=FL(-), M2=FR(+), M3=BL(-), M4=BR(+)
        cmd = Int32MultiArray()
        cmd.data = [-PWM_FULL, PWM_FULL, -PWM_FULL, PWM_FULL]
        self.motor_pub.publish(cmd)
        self.test_active = True
        self.start_time  = time.time()
        print(f"  🔄 Spinning LEFT at PWM={PWM_FULL}...")
        print(f"     Watch the reference mark on top of robot!")

        deadline = self.start_time + SPIN_DURATION
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        self._stop()
        self.test_active = False

    def analyse(self):
        print()
        print("="*55)
        print("  RESULTS — MAX ANGULAR VELOCITY")
        print("="*55)

        # METHOD A: encoder-based
        if self.velocities:
            steady = self.velocities[int(len(self.velocities)*0.3):]

            # Left wheels = M1(FL), M3(BL)  → moving backward → negative velocity
            # Right wheels = M2(FR), M4(BR) → moving forward  → positive velocity
            angular_samples = []
            for t, vels in steady:
                v_left  = (vels[0] + vels[2]) / 2.0   # FL + BL average
                v_right = (vels[1] + vels[3]) / 2.0   # FR + BR average
                # omega = (v_right - v_left) / wheelbase
                omega = (v_right - v_left) / WHEELBASE
                angular_samples.append(omega)

            avg_omega = sum(angular_samples) / len(angular_samples)
            max_omega = max(angular_samples)

            print(f"  METHOD A (encoder):")
            print(f"    Average angular vel : {avg_omega:.3f} rad/s")
            print(f"    Peak angular vel    : {max_omega:.3f} rad/s")
            print(f"    = {math.degrees(avg_omega):.1f} °/sec")
            print(f"    Time for 360°       : {360/math.degrees(avg_omega):.1f} seconds")
        else:
            print("  METHOD A: No encoder data received")
            avg_omega = None

        print()
        print("  METHOD B (manual stopwatch):")
        time_for_360 = input("    How many seconds did ONE full rotation take? → ")
        try:
            t360 = float(time_for_360)
            manual_omega = (2 * math.pi) / t360
            print(f"    Manual angular vel : {manual_omega:.3f} rad/s")
            print(f"    = {math.degrees(manual_omega):.1f} °/sec")
        except:
            print("    (skipped)")
            manual_omega = None

        print()
        print("  COMPARISON:")
        print(f"    Simulation uses  : 1.500 rad/s max")
        if avg_omega:
            print(f"    Encoder measured : {avg_omega:.3f} rad/s at PWM={PWM_FULL}")
            scale = avg_omega / PWM_FULL
            full_255 = scale * 255
            print(f"    Extrapolated PWM=255 : {full_255:.3f} rad/s")
        if manual_omega:
            print(f"    Manual measured  : {manual_omega:.3f} rad/s at PWM={PWM_FULL}")

        best = avg_omega or manual_omega
        if best:
            full = best * (255 / PWM_FULL)
            print()
            if full >= 1.3:
                print(f"  ✅ Robot CAN achieve ≥1.3 rad/s — simulation value OK")
                print(f"     Keep td3_env.py: ang × 1.50 rad/s")
            else:
                print(f"  ⚠  Max angular vel at PWM=255 ≈ {full:.2f} rad/s")
                print(f"     Update td3_inference.py: MAX_ANGULAR = {full:.2f}")
        print("="*55)


def main():
    rclpy.init()
    node = MaxAngularVelTest()

    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.analyse()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()