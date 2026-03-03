import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Bool
import time
import numpy as np
import math
import serial

class PIDControlNode(Node):
    def __init__(self):
        super().__init__('pid_control_node')

        self.subscription1 = self.create_subscription(
            Int32MultiArray, 'path_data', self.path_callback, 10
        )
        # Publisher for motor speeds
        try:
            self.arduino = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
            self.get_logger().info("Serial port opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port: {e}")
        self.publisher_bool = self.create_publisher(Bool, 'Completed', 10)

        # Desired speeds
        self.setspeed=0.5
        self.setpoints = [self.setspeed for i in range(4)]
        self.pos=[0,0]
        self.req_path=[0,0]
        self.curr_dir=1 #1:up, 2:right, 3:down, 4:left
        self.req_dir=1
        self.get_logger().info(f"PID_control_node started\nSet velocities: {self.setpoints}")
    
    def path_callback(self,msg):
        self.req_path=msg.data
        #self.req_path=[0,1]
        if self.req_path[0]-self.pos[0]==1:
            self.req_dir=1
        elif self.req_path[0]-self.pos[0]==-1:
            self.req_dir=3
        elif self.req_path[1]-self.pos[1]==1:
            self.req_dir=2
        else:
            self.req_dir=4
        if self.curr_dir!=self.req_dir:
            command=f"{180},{180},{-180},{-180}\n"
            self.arduino.write(command.encode('utf-8'))
            self.get_logger().info(f"Sent: {command.strip()}")
            time.sleep((0.040125*1.9*math.pi/self.setspeed)*(self.req_dir-self.curr_dir+(4 if self.req_dir<self.curr_dir else 0)))
            self.curr_dir=self.req_dir
        command=f"{88},{71},{83},{65}\n"
        self.arduino.write(command.encode('utf-8'))
        self.get_logger().info(f"Sent: {command.strip()}")
        time.sleep(1.1*0.25/self.setspeed)
        command=f"{0},{0},{0},{0}\n"
        self.arduino.write(command.encode('utf-8'))
        self.get_logger().info(f"Sent: {command.strip()}")
        lidar_msg=Bool()
        lidar_msg.data=True
        self.pos=self.req_path
        self.publisher_bool.publish(lidar_msg)
        self.get_logger().info(f"Completed msg sent to obstacle_detection node")

def main(args=None):
    rclpy.init(args=args)
    node = PIDControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

