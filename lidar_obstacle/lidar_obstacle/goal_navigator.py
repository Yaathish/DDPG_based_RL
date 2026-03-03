import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
import time

# Configuration
GOAL_X = 4.0
GOAL_Y = 0.0
GOAL_TOLERANCE = 0.2

# Motor speeds (0-250) - UPDATED MAX TO 250
BASE_SPEED = 200        # Normal forward speed
SLOW_SPEED = 150        # Slow forward speed
TURN_SPEED = 250        # Max speed for turning wheels
MIN_MOTOR_SPEED = 100   # Minimum speed that still moves the robot (increased from 45)

# Turn speed ratios
GENTLE_TURN_RATIO = 0.6  # Slow side = 60% of fast side (150/250 = 0.6)
# This gives: fast side = 250, slow side = 150 for gentle turns

# Movement parameters
METRES_PER_SEC = 0.18
DEGREES_PER_SEC = 20.0

# Obstacle detection distances (in meters)
OBSTACLE_DETECT_DIST = 0.5
EMERGENCY_DIST = 0.15
WALL_FOLLOW_DIST = 0.25

# Angular windows (degrees)
FRONT_ANGLE = 30
SIDE_ANGLE = 60

# Turning thresholds
HEADING_TOLERANCE = 15
MIN_AVOID_TIME = 1.0

# States
STATE_NAVIGATE = 'NAVIGATE'
STATE_AVOID = 'AVOID'
STATE_ARRIVED = 'ARRIVED'

class MotorMappedNavigator(Node):
    def __init__(self):
        super().__init__('motor_mapped_navigator')
        
        # Setup subscribers/publishers
        lidar_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, lidar_qos)
        self.motor_pub = self.create_publisher(Int32MultiArray, 'mot_val', 10)
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        
        # Obstacle information
        self.obstacle_zones = {
            'front': float('inf'),
            'front_left': float('inf'),
            'front_right': float('inf'),
            'left': float('inf'),
            'right': float('inf')
        }
        
        # State machine
        self.state = STATE_NAVIGATE
        self.avoid_direction = 'right'
        self.avoid_start_time = 0.0
        
        # Timing
        self.last_time = time.time()
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f"Motor Mapped Navigator started. Goal: ({GOAL_X}, {GOAL_Y})")
        self.get_logger().info("Motor mapping: [m1=BR, m2=BL, m3=FL, m4=FR]")
        self.get_logger().info(f"Speed settings: MAX=250, MIN={MIN_MOTOR_SPEED}, GENTLE_RATIO={GENTLE_TURN_RATIO}")

    def scan_callback(self, scan):
        """Process laser scan data"""
        # Reset zones
        for zone in self.obstacle_zones:
            self.obstacle_zones[zone] = float('inf')
        
        # Process each laser reading
        for i, dist in enumerate(scan.ranges):
            if not math.isfinite(dist) or dist <= 0.01:
                continue
            
            # Calculate angle in degrees
            angle_rad = scan.angle_min + i * scan.angle_increment
            angle_deg = math.degrees(angle_rad)
            angle_deg = (angle_deg + 180) % 360 - 180  # Normalize to -180 to 180
            
            # Classify into zones
            if abs(angle_deg) <= FRONT_ANGLE/2:
                # Front zone
                if dist < self.obstacle_zones['front']:
                    self.obstacle_zones['front'] = dist
            elif FRONT_ANGLE/2 < angle_deg <= FRONT_ANGLE/2 + SIDE_ANGLE:
                # Front right zone
                if dist < self.obstacle_zones['front_right']:
                    self.obstacle_zones['front_right'] = dist
            elif -(FRONT_ANGLE/2 + SIDE_ANGLE) <= angle_deg < -FRONT_ANGLE/2:
                # Front left zone
                if dist < self.obstacle_zones['front_left']:
                    self.obstacle_zones['front_left'] = dist
            elif angle_deg > FRONT_ANGLE/2 + SIDE_ANGLE:
                # Right side
                if dist < self.obstacle_zones['right']:
                    self.obstacle_zones['right'] = dist
            elif angle_deg < -(FRONT_ANGLE/2 + SIDE_ANGLE):
                # Left side
                if dist < self.obstacle_zones['left']:
                    self.obstacle_zones['left'] = dist

    def publish_motors(self, m1, m2, m3, m4, label):
        """
        Publish motor values with correct mapping:
        m1 = Back Right (BR)
        m2 = Back Left (BL)
        m3 = Front Left (FL)
        m4 = Front Right (FR)
        """
        # Ensure values are within -250 to 250 range
        m1 = max(-250, min(250, m1))
        m2 = max(-250, min(250, m2))
        m3 = max(-250, min(250, m3))
        m4 = max(-250, min(250, m4))
        
        msg = Int32MultiArray()
        msg.data = [m1, m2, m3, m4]
        self.motor_pub.publish(msg)
        
        # Log movement with motor mapping info
        self.get_logger().info(
            f"[{label:15s}] pos=({self.x:.2f},{self.y:.2f}) "
            f"hdg={self.heading:.1f}° goal={self.angle_to_goal():.1f}° "
            f"obs={self.obstacle_zones['front']:.2f}m "
            f"motors[BR={m1:4d}, BL={m2:4d}, FL={m3:4d}, FR={m4:4d}]"
        )

    def stop(self):
        """Stop all motors"""
        self.publish_motors(0, 0, 0, 0, "STOP")

    def move_forward(self, speed=BASE_SPEED):
        """
        Move forward:
        - All wheels turn same direction (positive = forward)
        - BR(m1)=+, BL(m2)=+, FL(m3)=+, FR(m4)=+
        """
        self.publish_motors(speed, speed, speed, speed, f"FWD({speed})")

    def move_backward(self, speed=BASE_SPEED):
        """
        Move backward:
        - All wheels reverse (negative = backward)
        - BR(m1)=-speed, BL(m2)=-speed, FL(m3)=-speed, FR(m4)=-speed
        """
        self.publish_motors(-speed, -speed, -speed, -speed, f"REV({speed})")

    def turn_left_sharp(self, speed=TURN_SPEED):
        """
        Sharp left turn (spin in place):
        - Left side wheels reverse, Right side wheels forward
        - BR(m1)=+speed (right side forward)
        - BL(m2)=-speed (left side reverse)
        - FL(m3)=-speed (left side reverse)
        - FR(m4)=+speed (right side forward)
        """
        self.publish_motors(speed, -speed, -speed, speed, f"LEFT-SHARP({speed})")

    def turn_right_sharp(self, speed=TURN_SPEED):
        """
        Sharp right turn (spin in place):
        - Right side wheels reverse, Left side wheels forward
        - BR(m1)=-speed (right side reverse)
        - BL(m2)=+speed (left side forward)
        - FL(m3)=+speed (left side forward)
        - FR(m4)=-speed (right side reverse)
        """
        self.publish_motors(-speed, speed, speed, -speed, f"RIGHT-SHARP({speed})")

    def turn_left_gentle(self, speed=TURN_SPEED):
        """
        Gentle left turn (moving forward while turning):
        - Left side slower, Right side faster
        - BR(m1)=+speed (right side fast)
        - BL(m2)=+slow (left side slow)
        - FL(m3)=+slow (left side slow)
        - FR(m4)=+speed (right side fast)
        Where slow = speed * GENTLE_TURN_RATIO (ensuring it's >= MIN_MOTOR_SPEED)
        """
        slow = int(speed * GENTLE_TURN_RATIO)
        # Ensure slow side doesn't go below minimum
        if slow < MIN_MOTOR_SPEED and speed > MIN_MOTOR_SPEED:
            slow = MIN_MOTOR_SPEED
        
        self.publish_motors(speed, slow, slow, speed, f"LEFT-GENTLE({speed}/{slow})")

    def turn_right_gentle(self, speed=TURN_SPEED):
        """
        Gentle right turn (moving forward while turning):
        - Right side slower, Left side faster
        - BR(m1)=+slow (right side slow)
        - BL(m2)=+speed (left side fast)
        - FL(m3)=+speed (left side fast)
        - FR(m4)=+slow (right side slow)
        Where slow = speed * GENTLE_TURN_RATIO (ensuring it's >= MIN_MOTOR_SPEED)
        """
        slow = int(speed * GENTLE_TURN_RATIO)
        # Ensure slow side doesn't go below minimum
        if slow < MIN_MOTOR_SPEED and speed > MIN_MOTOR_SPEED:
            slow = MIN_MOTOR_SPEED
        
        self.publish_motors(slow, speed, speed, slow, f"RIGHT-GENTLE({speed}/{slow})")

    def pivot_left(self, speed=TURN_SPEED):
        """
        Pivot left (turn around left wheels):
        - Left wheels stopped, Right wheels forward
        - BR(m1)=+speed (right side forward)
        - BL(m2)=0 (left side stopped)
        - FL(m3)=0 (left side stopped)
        - FR(m4)=+speed (right side forward)
        """
        self.publish_motors(speed, 0, 0, speed, f"LEFT-PIVOT({speed})")

    def pivot_right(self, speed=TURN_SPEED):
        """
        Pivot right (turn around right wheels):
        - Right wheels stopped, Left wheels forward
        - BR(m1)=0 (right side stopped)
        - BL(m2)=+speed (left side forward)
        - FL(m3)=+speed (left side forward)
        - FR(m4)=0 (right side stopped)
        """
        self.publish_motors(0, speed, speed, 0, f"RIGHT-PIVOT({speed})")

    def update_odometry(self, dt, command):
        """Update position based on motor command"""
        if 'FWD' in command:
            # Forward movement
            d = METRES_PER_SEC * dt
            self.x += d * math.cos(math.radians(self.heading))
            self.y += d * math.sin(math.radians(self.heading))
        elif 'REV' in command:
            # Reverse movement
            d = METRES_PER_SEC * dt * 0.5
            self.x -= d * math.cos(math.radians(self.heading))
            self.y -= d * math.sin(math.radians(self.heading))
        
        # Turning calculations
        turn_rate = 0
        if 'SHARP' in command:
            turn_rate = DEGREES_PER_SEC * 2.0
        elif 'GENTLE' in command:
            turn_rate = DEGREES_PER_SEC * 0.5
        elif 'PIVOT' in command:
            turn_rate = DEGREES_PER_SEC * 1.5
        
        if 'LEFT' in command:
            self.heading += turn_rate * dt
        elif 'RIGHT' in command:
            self.heading -= turn_rate * dt
        
        # Normalize heading
        self.heading = (self.heading + 180) % 360 - 180

    def angle_to_goal(self):
        """Calculate angle to goal relative to current heading"""
        dx = GOAL_X - self.x
        dy = GOAL_Y - self.y
        goal_angle = math.degrees(math.atan2(dy, dx))
        diff = goal_angle - self.heading
        return (diff + 180) % 360 - 180

    def distance_to_goal(self):
        """Calculate Euclidean distance to goal"""
        return math.hypot(GOAL_X - self.x, GOAL_Y - self.y)

    def is_obstacle_ahead(self):
        """Check if there's an obstacle directly ahead"""
        return self.obstacle_zones['front'] < OBSTACLE_DETECT_DIST

    def get_clear_direction(self):
        """Determine which direction is clearer for turning"""
        left_clearance = min(self.obstacle_zones['left'], 
                           self.obstacle_zones['front_left'])
        right_clearance = min(self.obstacle_zones['right'], 
                            self.obstacle_zones['front_right'])
        
        # If both sides are clear, prefer direction toward goal
        if left_clearance > OBSTACLE_DETECT_DIST and right_clearance > OBSTACLE_DETECT_DIST:
            goal_angle = self.angle_to_goal()
            if abs(goal_angle) < 30:
                return 'forward'
            return 'left' if goal_angle > 0 else 'right'
        
        # Choose side with more clearance
        if left_clearance > right_clearance + 0.1:
            return 'left'
        elif right_clearance > left_clearance + 0.1:
            return 'right'
        else:
            return self.avoid_direction

    def control_loop(self):
        """Main control loop"""
        now = time.time()
        dt = min(now - self.last_time, 0.2)
        self.last_time = now
        
        # Check if goal reached
        if self.distance_to_goal() < GOAL_TOLERANCE:
            self.stop()
            self.state = STATE_ARRIVED
            self.get_logger().info(f"GOAL REACHED! Final: ({self.x:.2f}, {self.y:.2f})")
            return
        
        if self.state == STATE_ARRIVED:
            return
        
        # Emergency stop - but only if REALLY close
        if self.obstacle_zones['front'] < EMERGENCY_DIST:
            self.stop()
            self.get_logger().warn(f"EMERGENCY STOP! Obstacle at {self.obstacle_zones['front']:.2f}m")
            # Back up a little
            if self.state != STATE_AVOID:
                self.move_backward(SLOW_SPEED)
                self.update_odometry(dt, "REV")
                self.state = STATE_AVOID
                self.avoid_start_time = now
            return
        
        # State machine
        if self.state == STATE_NAVIGATE:
            if self.is_obstacle_ahead():
                # Obstacle detected - start avoiding
                self.state = STATE_AVOID
                self.avoid_start_time = now
                self.avoid_direction = self.get_clear_direction()
                self.get_logger().info(f"Obstacle ahead -> AVOID (turning {self.avoid_direction})")
            else:
                # Normal navigation toward goal
                goal_angle = self.angle_to_goal()
                
                if abs(goal_angle) > HEADING_TOLERANCE:
                    # Need to turn toward goal
                    if goal_angle > 0:
                        self.turn_left_gentle()
                        self.update_odometry(dt, "LEFT-GENTLE")
                    else:
                        self.turn_right_gentle()
                        self.update_odometry(dt, "RIGHT-GENTLE")
                else:
                    # Heading correct - go forward
                    self.move_forward()
                    self.update_odometry(dt, "FWD")
        
        elif self.state == STATE_AVOID:
            elapsed = now - self.avoid_start_time
            
            # Check if path is clear
            front_clear = self.obstacle_zones['front'] > OBSTACLE_DETECT_DIST * 1.5
            
            if front_clear and elapsed > MIN_AVOID_TIME:
                # Path is clear, return to navigation
                self.state = STATE_NAVIGATE
                self.get_logger().info("Path clear -> NAVIGATE")
            else:
                # Continue avoiding - use sharper turns based on obstacle proximity
                if self.obstacle_zones['front'] < EMERGENCY_DIST * 2:
                    # Very close - sharp turn
                    if self.avoid_direction == 'left':
                        self.turn_left_sharp()
                        self.update_odometry(dt, "LEFT-SHARP")
                    else:
                        self.turn_right_sharp()
                        self.update_odometry(dt, "RIGHT-SHARP")
                elif self.obstacle_zones['front'] < OBSTACLE_DETECT_DIST:
                    # Moderately close - pivot turn
                    if self.avoid_direction == 'left':
                        self.pivot_left()
                        self.update_odometry(dt, "LEFT-PIVOT")
                    else:
                        self.pivot_right()
                        self.update_odometry(dt, "RIGHT-PIVOT")
                else:
                    # Further away - gentle turn while moving forward
                    if self.avoid_direction == 'left':
                        self.turn_left_gentle()
                        self.update_odometry(dt, "LEFT-GENTLE")
                    else:
                        self.turn_right_gentle()
                        self.update_odometry(dt, "RIGHT-GENTLE")

def main(args=None):
    rclpy.init(args=args)
    node = MotorMappedNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()