"""
TEST 4 — Motor Direction Verification
========================================
PURPOSE:
  Confirms that positive PWM = forward motion for ALL 4 wheels.
  Wrong motor wiring = robot spins/drifts instead of going straight.
  Also confirms encoder sign matches motor direction.

HOW IT WORKS:
  Tests each motor individually at low PWM for 1 second.
  You observe each wheel and say if it spins forward or backward.
  Script tells you which motors need sign correction in Arduino sketch.

ROBOT LAYOUT (looking from above):
  ┌─────────────────────┐
  │  M1(FL)    M2(FR)   │  ← FRONT
  │                     │
  │  M3(BL)    M4(BR)   │  ← BACK
  └─────────────────────┘
  
  For FORWARD motion:
    Left wheels (M1, M3):  should spin CLOCKWISE      (when viewed from left side)
    Right wheels (M2, M4): should spin COUNTER-CLOCKWISE (when viewed from right side)

  If any wheel spins wrong: note it — update Arduino sketch motor direction

SETUP:
  1. Lift robot OFF the ground (place on a box/book)
  2. Motor power ON

PLACE IN REPO:
  motor_node/motor_node/test4_motor_direction.py

RUN:
  Terminal 1: ros2 run encoder_node encoder_pub
  Terminal 2: ros2 run motor_node motor_sub
  Terminal 3: python3 test4_motor_direction.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import time

PWM_TEST = 120   # Low speed for safe observation

MOTOR_NAMES = ['M1 (Front-Left)', 'M2 (Front-Right)',
               'M3 (Back-Left)',  'M4 (Back-Right)']

class MotorDirectionTest(Node):
    def __init__(self):
        super().__init__('test4_motor_direction')

        self.motor_pub = self.create_publisher(Int32MultiArray, 'mot_val', 10)
        self.enc_sub   = self.create_subscription(
            Float32MultiArray, 'enc_val', self._enc_cb, 10
        )
        self.enc_data = []

        print("\n" + "="*55)
        print("  TEST 4 — MOTOR DIRECTION VERIFICATION")
        print("="*55)
        print()
        print("  ROBOT LAYOUT (viewed from above):")
        print("  ┌─────────────────────┐")
        print("  │  M1(FL)    M2(FR)   │  ← FRONT")
        print("  │                     │")
        print("  │  M3(BL)    M4(BR)   │  ← BACK")
        print("  └─────────────────────┘")
        print()
        print("  REQUIREMENTS:")
        print("    ✅ Robot lifted OFF ground (on a box)")
        print("    ✅ encoder_pub running")
        print("    ✅ motor_sub running")
        print()
        input("  → Press ENTER to start motor tests...")

    def _enc_cb(self, msg):
        self.enc_data = list(msg.data)

    def _stop(self):
        msg = Int32MultiArray()
        msg.data = [0, 0, 0, 0]
        for _ in range(5):
            self.motor_pub.publish(msg)
            time.sleep(0.05)

    def _spin_one(self, motor_idx, pwm):
        """Spin one motor at given PWM, others at 0."""
        cmd = Int32MultiArray()
        vals = [0, 0, 0, 0]
        vals[motor_idx] = pwm
        cmd.data = vals
        self.motor_pub.publish(cmd)

    def _get_enc_vel(self, motor_idx):
        """Get current velocity for one motor."""
        if self.enc_data and len(self.enc_data) > motor_idx:
            return self.enc_data[motor_idx]
        return 0.0

    def run(self):
        results = []

        for i, name in enumerate(MOTOR_NAMES):
            print()
            print(f"  ─── Testing {name} ───")
            print(f"  Sending PWM = +{PWM_TEST} (should spin FORWARD)")
            print(f"  Watch this wheel carefully...")
            print()

            # Run motor
            self._spin_one(i, PWM_TEST)
            time.sleep(0.5)   # let it spin up

            # Read encoder
            enc_vels = []
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.05)
                enc_vels.append(self._get_enc_vel(i))

            avg_enc = sum(enc_vels) / len(enc_vels)

            self._stop()
            time.sleep(0.3)

            # Ask user what they observed
            print(f"  Encoder reading: {avg_enc:.3f} m/s")
            direction = input(f"  Did {name} spin FORWARD? (y/n/not-moving): ").strip().lower()
            results.append({
                'motor': name,
                'idx': i,
                'pwm': PWM_TEST,
                'enc_vel': avg_enc,
                'forward': direction == 'y',
                'moving': direction != 'not-moving'
            })

        self._stop()
        self._report(results)

    def _report(self, results):
        print()
        print("="*55)
        print("  RESULTS — MOTOR DIRECTIONS")
        print("="*55)
        print()

        corrections_needed = []
        for r in results:
            status = "✅ CORRECT" if r['forward'] else ("❌ REVERSED" if r['moving'] else "⚠ NOT MOVING")
            print(f"  {r['motor']:20s}  enc={r['enc_vel']:+.3f}  {status}")
            if not r['forward'] and r['moving']:
                corrections_needed.append(r['idx'])

        print()
        if not corrections_needed:
            print("  ✅ ALL MOTORS CORRECT — no changes needed")
        else:
            print("  ❌ Motors needing direction fix in Arduino sketch:")
            for idx in corrections_needed:
                print(f"     {MOTOR_NAMES[idx]}: reverse motor wiring OR negate PWM in Arduino")
            print()
            print("  ARDUINO FIX: in your Arduino sketch, for each wrong motor,")
            print("  negate the value before writing to motor shield:")
            print("  e.g.  motorSpeed[1] = -motorSpeed[1];  // flip M2")

        print()
        print("  NOW TEST ALL-FORWARD:")
        input("  → Place robot on floor. Press ENTER to run all forward for 2 sec...")

        cmd = Int32MultiArray()
        cmd.data = [PWM_TEST, PWM_TEST, PWM_TEST, PWM_TEST]
        self.motor_pub.publish(cmd)
        print("  🚀 All motors forward...")
        time.sleep(2.0)
        self._stop()

        straight = input("  Did robot go STRAIGHT forward? (y/n): ").strip().lower()
        if straight == 'y':
            print("  ✅ Robot drives straight — motors are correctly configured")
        else:
            print("  ⚠  Robot drifts — motor speeds need balancing")
            print("     Note which side it drifts toward for PID tuning")

        print("="*55)


def main():
    rclpy.init()
    node = MotorDirectionTest()

    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()