"""
TEST 5 — Physical Measurements Guide + Update Summary
=======================================================
PURPOSE:
  Guide for ruler measurements (wheelbase, wheel diameter)
  and a summary of WHERE to update all values in your codebase.

  Run this AFTER tests 1-4 to get a complete update checklist.

NO ROS2 NEEDED — just run: python3 test5_update_summary.py

PLACE IN REPO:
  motor_node/motor_node/test5_update_summary.py
"""

print("""
╔══════════════════════════════════════════════════════╗
║     PHYSICAL MEASUREMENTS GUIDE                      ║
╠══════════════════════════════════════════════════════╣

  MEASUREMENT 1 — WHEELBASE
  ─────────────────────────
  Definition: Distance between LEFT wheel centre and RIGHT wheel centre
  How to measure:
    1. Place ruler across the robot
    2. Measure from centre of left front wheel
       to centre of right front wheel
    3. Record in metres (e.g. 155mm = 0.155m)
  
  Current value in code: 0.155m
  Check in:
    → encoder_pub.py  (not used directly but affects odometry)
    → td3_env.py      (WHEELBASE not used — skid steer model)
    → test3 above     (WHEELBASE = 0.155 → update if different)
    → td3_inference.py (if sensor_fusion.py uses it)

  ┌───────────────────────────┐
  │  ←←← WHEELBASE →→→      │
  │  [L wheel]   [R wheel]   │
  └───────────────────────────┘

  MEASUREMENT 2 — WHEEL DIAMETER
  ────────────────────────────────
  How to measure:
    1. Lay a ruler next to the wheel
    2. Measure from bottom to top of rubber tyre
    3. Record in metres (e.g. 65mm = 0.065m)
  
  Current value in code: 0.065m
  Check in:
    → encoder_pub.py  line: self.diameter = 0.065
    → td3_env.py      (not directly used)
  
  If different from 0.065m:
    Update encoder_pub.py: self.diameter = YOUR_VALUE


╠══════════════════════════════════════════════════════╣
║     WHERE TO UPDATE ALL MEASURED VALUES              ║
╠══════════════════════════════════════════════════════╣

  After running all tests, fill in this table:

  ┌─────────────────────┬──────────────┬──────────────┐
  │ Parameter           │ Current Code │ Your Measured│
  ├─────────────────────┼──────────────┼──────────────┤
  │ PPR                 │ 20           │ _____        │
  │ Wheel diameter (m)  │ 0.065        │ _____        │
  │ Wheelbase (m)       │ 0.155        │ _____        │
  │ Max linear vel(m/s) │ 0.300        │ _____        │
  │ Max angular vel     │ 1.500 rad/s  │ _____        │
  └─────────────────────┴──────────────┴──────────────┘

  UPDATE LOCATIONS:

  1. PPR  →  encoder_node/encoder_node/encoder_pub.py
     Line: self.ppr = 20
     Change to: self.ppr = YOUR_VALUE

  2. Wheel diameter  →  encoder_node/encoder_node/encoder_pub.py  
     Line: self.diameter = 0.065
     Change to: self.diameter = YOUR_VALUE

  3. Max linear velocity  →  lidar_obstacle/lidar_obstacle/td3_inference.py
     (file to be added — see below)
     Line: MAX_LINEAR = 0.30
     Change to: MAX_LINEAR = YOUR_VALUE

  4. Max angular velocity  →  lidar_obstacle/lidar_obstacle/td3_inference.py
     Line: MAX_ANGULAR = 1.50
     Change to: MAX_ANGULAR = YOUR_VALUE

  5. Wheelbase  →  lidar_obstacle/lidar_obstacle/sensor_fusion.py
     (if using odometry fusion)
     Line: WHEELBASE = 0.155
     Change to: YOUR_VALUE


╠══════════════════════════════════════════════════════╣
║     DEPLOYMENT CHECKLIST                             ║
╠══════════════════════════════════════════════════════╣

  Before copying trained model to Pi 5:

  □ Test 1 PPR verified and encoder_pub.py updated
  □ Test 2 max linear velocity measured
  □ Test 3 max angular velocity measured  
  □ Test 4 all motors spin correct direction
  □ Wheelbase measured with ruler
  □ Wheel diameter measured with ruler
  □ td3_inference.py values match real robot
  □ Robot drives straight at equal PWM
  □ Model file copied: TD3_actor.pth → Pi 5
  □ actor_critic.py copied to Pi 5

╚══════════════════════════════════════════════════════╝
""")

# Interactive update checker
print("  QUICK CHECK — enter your measured values:")
print("  (Press ENTER to skip if not measured yet)")
print()

vals = {}

ppr = input("  PPR from test 1: ").strip()
if ppr:
    vals['ppr'] = float(ppr)
    current = 20
    if abs(float(ppr) - current) > 1:
        print(f"  ⚠  UPDATE encoder_pub.py: self.ppr = {round(float(ppr))}")
    else:
        print(f"  ✅ PPR={ppr} matches current value of 20")

diam = input("  Wheel diameter (mm, e.g. 65): ").strip()
if diam:
    d_m = float(diam) / 1000
    if abs(d_m - 0.065) > 0.002:
        print(f"  ⚠  UPDATE encoder_pub.py: self.diameter = {d_m:.3f}")
    else:
        print(f"  ✅ Diameter matches 0.065m")

wb = input("  Wheelbase (mm, e.g. 155): ").strip()
if wb:
    wb_m = float(wb) / 1000
    if abs(wb_m - 0.155) > 0.005:
        print(f"  ⚠  UPDATE sensor_fusion.py and test3: WHEELBASE = {wb_m:.3f}")
    else:
        print(f"  ✅ Wheelbase matches 0.155m")

maxlin = input("  Max linear velocity (m/s from test 2): ").strip()
if maxlin:
    v = float(maxlin)
    if abs(v - 0.30) > 0.03:
        print(f"  ⚠  UPDATE td3_inference.py: MAX_LINEAR = {v:.2f}")
    else:
        print(f"  ✅ Max linear vel matches simulation (0.30 m/s)")

maxang = input("  Max angular velocity (rad/s from test 3): ").strip()
if maxang:
    w = float(maxang)
    if abs(w - 1.50) > 0.15:
        print(f"  ⚠  UPDATE td3_inference.py: MAX_ANGULAR = {w:.2f}")
    else:
        print(f"  ✅ Max angular vel matches simulation (1.50 rad/s)")

print()
print("  ✅ Run complete. Apply any ⚠ changes before deploying model.")
print()