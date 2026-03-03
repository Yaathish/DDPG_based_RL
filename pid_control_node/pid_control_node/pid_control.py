import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Bool
from time import time
import numpy as np
import math

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time()

    def compute(self, setpoint, measured_value):
        current_time = time()
        dt = current_time - self.last_time
        self.last_time = current_time

        if dt <= 0.0:  # Prevent division by zero
            dt = 1e-3  # Small default dt

        error = setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.previous_error = error

        return output

    def update_gains(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd


class PIDControlNode(Node):
    def __init__(self):
        super().__init__('pid_control_node')

        # Subscribe to encoder values
        self.subscription = self.create_subscription(
            Float32MultiArray, 'enc_val', self.encoder_callback, 10
        )
        
        self.subscription1 = self.create_subscription(
            Int32MultiArray, 'path_data', self.path_callback, 10
        )
        # Publisher for motor speeds
        self.publisher = self.create_publisher(Int32MultiArray, 'mot_val', 10)
        self.publisher_bool = self.create_publisher(Bool, 'Completed', 10)

        # Initialize PID controllers (4 wheels)
        self.pid_controllers = [PIDController(kp=0.7, ki=0.3, kd=0) for _ in range(4)]

        # Desired speeds
        self.setspeed=0.8
        self.setpoints = [self.setspeed for i in range(4)]
        self.encoder_values = [0, 0, 0, 0]

        # Parameters for tuning PID
        self.declare_parameter('kp', 0.7)
        self.declare_parameter('ki', 0.3)
        self.declare_parameter('kd', 0.0)
        self.dt=0.03
        self.pos=[0,0,0]
        self.stop=False
        self.req_path=[1,0]
        self.rotated=True
        self.curr_dir=1 #1:up, 2:right, 3:down, 4:left
        self.req_dir=1
        self.curr_dir_radians=0
        self.req_dir_radians=0
        self.velocities=[[1.1026990214100174, 1.158854990092935, 1.1537499020308517, 1.158854990092935, 1.1486448139687682, 1.1384346378446013, 1.133329549782518, 1.1282244617204344, 1.133329549782518, 1.1231193736583511, 1.133329549782518, 1.1282244617204344, 1.1231193736583511, 1.1282244617204344, 1.1384346378446013, 1.1231193736583511, 1.1180142855962676, 1.1129091975341843, 1.107804109472101, 1.1026990214100174, 1.097593933347934, 1.1026990214100174, 1.097593933347934, 1.097593933347934, 1.0924888452858506, 1.0924888452858506, 1.1026990214100174, 1.0924888452858506, 1.0924888452858506, 1.0873837572237672, 1.0924888452858506, 1.0873837572237672, 1.0873837572237672, 1.0873837572237672, 1.0822786691616837, 1.0822786691616837, 1.0771735810996004, 1.0720684930375168, 1.0771735810996004, 1.0669634049754335, 1.0669634049754335, 1.0516481407891833, 1.0516481407891833, 1.0465430527271, 1.036332876602933, 1.036332876602933, 1.036332876602933, 1.0312277885408496, 1.026122700478766, 1.0210176124166828, 1.0108074362925161, 1.0159125243545994, 1.0210176124166828, 1.0057023482304326, 1.0057023482304326, 0.9954921721062657, 0.9903870840441824, 0.9852819959820989, 0.9801769079200155, 0.9699667317958486, 0.9648616437337653, 0.9546514676095985, 0.949546379547515, 0.949546379547515, 0.9291260272991814, 0.9189158511750145, 0.9087056750508478, 0.9036005869887642, 0.8933904108645974, 0.8831802347404306, 0.8678649705541804, 0.8576547944300137, 0.8474446183058466, 0.8474446183058466, 0.8474446183058466, 0.8321293541195965, 0.8219191779954297, 0.8117090018712629, 0.8014988257470961, 0.7912886496229292, 0.7810784734987624, 0.7606581212504288, 0.7555530331883453, 0.740237769002095, 0.7300275928779282, 0.714712328691678, 0.6993970645054277, 0.6840818003191774, 0.6738716241950107, 0.6585563600087604, 0.6381360077604268, 0.6228207435741766, 0.6075054793879262, 0.5870851271395926, 0.566664774891259, 0.5564545987670921, 0.5309291584566751, 0.5156138942704248, 0.5054037181462581, 0.4849833658979243, 0.4645630136495907, 0.44924774946334045, 0.41861722109083993, 0.4084070449666731, 0.37267142853208923, 0.35225107628375557, 0.3216205479112551, 0.2858849314766712, 0.23993913891792046, 0.20420352248333656, 0.16336281798666924, 0.12762720155208535, 0.08678649705541805, 0.06636614480708439, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [1.0210176124166828, 1.0210176124166828, 1.0210176124166828, 1.0210176124166828, 0.9189158511750145, 1.0210176124166828, 1.0210176124166828, 1.0210176124166828, 0.9189158511750145, 1.0210176124166828, 1.0210176124166828, 0.9189158511750145, 1.0210176124166828, 0.9189158511750145, 0.9189158511750145, 1.0210176124166828, 0.9189158511750145, 0.9189158511750145, 0.9189158511750145, 0.9189158511750145, 0.9189158511750145, 0.9189158511750145, 0.9189158511750145, 0.9189158511750145, 0.8168140899333463, 0.9189158511750145, 0.9189158511750145, 0.8168140899333463, 0.9189158511750145, 0.8168140899333463, 0.8168140899333463, 0.9189158511750145, 0.8168140899333463, 0.8168140899333463, 0.8168140899333463, 0.8168140899333463, 0.8168140899333463, 0.8168140899333463, 0.8168140899333463, 0.8168140899333463, 0.714712328691678, 0.8168140899333463, 0.714712328691678, 0.714712328691678, 0.8168140899333463, 0.714712328691678, 0.714712328691678, 0.714712328691678, 0.714712328691678, 0.714712328691678, 0.714712328691678, 0.6126105674500097, 0.714712328691678, 0.6126105674500097, 0.6126105674500097, 0.714712328691678, 0.6126105674500097, 0.6126105674500097, 0.6126105674500097, 0.6126105674500097, 0.5105088062083414, 0.6126105674500097, 0.5105088062083414, 0.6126105674500097, 0.5105088062083414, 0.5105088062083414, 0.5105088062083414, 0.5105088062083414, 0.5105088062083414, 0.4084070449666731, 0.5105088062083414, 0.4084070449666731, 0.4084070449666731, 0.5105088062083414, 0.3063052837250049, 0.4084070449666731, 0.4084070449666731, 0.4084070449666731, 0.3063052837250049, 0.3063052837250049, 0.4084070449666731, 0.3063052837250049, 0.3063052837250049, 0.3063052837250049, 0.20420352248333656, 0.20420352248333656, 0.20420352248333656, 0.3063052837250049, 0.20420352248333656, 0.10210176124166828, 0.20420352248333656, 0.10210176124166828, 0.20420352248333656, 0.10210176124166828, 0.0, 0.10210176124166828, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [1.010807436292516, 0.9971938681269602, 1.000597260168349, 1.004000652209738, 0.9971938681269602, 0.9903870840441823, 0.9801769079200154, 0.9767735158786266, 0.9699667317958487, 0.9631599477130709, 0.949546379547515, 0.9461429875061261, 0.9359328113819593, 0.9257226352577924, 0.9223192432164035, 0.9155124591336256, 0.9121090670922366, 0.8984954989266808, 0.891688714843903, 0.8746717546369583, 0.8746717546369583, 0.8644615785127915, 0.8576547944300136, 0.8440412262644579, 0.8304276580989021, 0.8134106978919573, 0.7963937376850125, 0.7929903456436237, 0.769166601353901, 0.7623598172711232, 0.7487462491055674, 0.7317292888986227, 0.7215191127744558, 0.6942919764433443, 0.6806784082777886, 0.6670648401122328, 0.6500478799052881, 0.6228207435741766, 0.6092071754086207, 0.595593607243065, 0.5785766470361203, 0.5547529027463977, 0.5377359425394529, 0.5139121982497303, 0.4900884539600077, 0.46626470967028516, 0.4458443574219515, 0.41521382904945103, 0.39479347680111737, 0.3743731245527837, 0.34714598822167214, 0.3165154598491717, 0.2824815394352822, 0.25865779514555964, 0.2246238747316702, 0.18718656227639185, 0.1361356816555577, 0.05445427266222309, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.9648616437337653, 1.0465430527271, 1.0567532288512669, 1.0516481407891833, 1.0465430527271, 1.0414379646650165, 1.036332876602933, 1.026122700478766, 1.0159125243545994, 1.0210176124166828, 1.0159125243545994, 1.0159125243545994, 1.0057023482304326, 1.0005972601683493, 1.0005972601683493, 0.9750718198579321, 0.9750718198579321, 0.9699667317958486, 0.9597565556716818, 0.9444412914854317, 0.9342311153612648, 0.9291260272991814, 0.924020939237098, 0.9342311153612648, 0.9291260272991814, 0.924020939237098, 0.9087056750508478, 0.9138107631129311, 0.9087056750508478, 0.8984954989266809, 0.8984954989266809, 0.8984954989266809, 0.8933904108645974, 0.8831802347404306, 0.8933904108645974, 0.8831802347404306, 0.8729700586162638, 0.8780751466783473, 0.862759882492097, 0.8525497063679301, 0.8474446183058466, 0.8372344421816799, 0.827024266057513, 0.827024266057513, 0.8168140899333463, 0.8117090018712629, 0.8014988257470961, 0.7963937376850125, 0.7810784734987624, 0.7708682973745955, 0.7759733854366789, 0.7657632093125122, 0.7453428570641785, 0.7351326809400116, 0.7351326809400116, 0.7351326809400116, 0.714712328691678, 0.7096072406295946, 0.6993970645054277, 0.6840818003191774, 0.6585563600087604, 0.6381360077604268, 0.6483461838845936, 0.6432410958225102, 0.6279258316362599, 0.6228207435741766, 0.6024003913258429, 0.5870851271395926, 0.5768749510154259, 0.5615596868291756, 0.5513495107050087, 0.5360342465187584, 0.5258240703945917, 0.5054037181462581, 0.5002986300841746, 0.4798782778358409, 0.46966810171167406, 0.4543528375254239, 0.43903757333917365, 0.41861722109083993, 0.4084070449666731, 0.3930917807804229, 0.37267142853208923, 0.357356164345839, 0.3420409001595887, 0.3267256359733385, 0.3063052837250048, 0.296095107600838, 0.27567475535250435, 0.26546457922833755, 0.23993913891792046, 0.22972896279375363, 0.21441369860750342, 0.19399334635916973, 0.1786780821729195, 0.16846790604875267, 0.1531526418625024, 0.13783737767625218, 0.11741702542791851, 0.10720684930375171, 0.08168140899333462, 0.0714712328691678, 0.06126105674500097, 0.056155968682917556, 0.045945792558750725, 0.045945792558750725, 0.030630528372500486, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
        self.sizes=[134,124,90,134] #sizes of each array in velocities(total 4 arrays for each motor)
        self.get_logger().info(f"PID_control_node started\nSet velocities: {self.setpoints}")
        self.timer = self.create_timer(self.dt, self.control_loop_callback)
        #self.timer_param = self.create_timer(1.0, self.update_pid_gains)
    
    def path_callback(self,msg):
        self.stop=False
        self.req_path=msg.data
        #self.req_path=[0,1]
        self.req_path=[self.req_path[0]/4 , self.req_path[1]/4]
        if self.req_path[0]-self.pos[0]==0.25:
            self.req_dir=1
        elif self.req_path[0]-self.pos[0]==-0.25:
            self.req_dir=3
        elif self.req_path[1]-self.pos[1]==0.25:
            self.req_dir=2
        else:
            self.req_dir=4
        if self.curr_dir!=self.req_dir:
            self.rotated=False
            self.req_dir_radians=(self.req_dir-self.curr_dir)*math.pi/2
        
    def encoder_callback(self, msg):
        """ Update encoder values from the message. """
        if len(msg.data) == 4:
            self.encoder_values = msg.data
        else:
            self.get_logger().error("Received encoder data with incorrect size!")

    def control_loop_callback(self):
        """ PID control loop running at a fixed rate. """
        if not self.stop:
            pwm=[0,0,0,0]
            [l1,l2,r1,r2]=self.encoder_values
            [x,y,theta]=self.pos
            v=(sum(self.encoder_values))/4 
            omega=((l1+l2)-(r1+r2))/0.321
            self.pos=[x+v*math.cos(theta)*self.dt,y+v*math.sin(theta)*self.dt,theta+omega*self.dt if -2*math.pi<theta+omega*self.dt<2*math.pi else (theta+omega*self.dt - 2*math.pi if theta+omega*self.dt>2*math.pi else theta+omega*self.dt+2*math.pi)]
            self.get_logger().info(f"Current position: {self.pos}")
            if not self.rotated:
                self.setpoints = [self.setspeed, self.setspeed, -self.setspeed, -self.setspeed]
                self.get_logger().info(f"{self.req_dir_radians},{self.curr_dir_radians}")
                self.curr_dir_radians+=omega*self.dt
                if abs(self.curr_dir_radians-self.req_dir_radians)<0.05 or self.curr_dir_radians>=self.req_dir_radians:
                    self.rotated=True
                    self.curr_dir=self.req_dir
                    self.curr_dir_radians=0
                    return
            else:
                self.setpoints=[0.7,0.7,-0.7,-0.7]            
                if abs(self.pos[0]-self.req_path[0])<0.05 and abs(self.pos[1]-self.req_path[1])<0.05:
                    self.stop=True
                    #self.pos[0]=self.req_path[0]
                    #self.pos[1]=self.req_path[1]
                    lidar_msg=Bool()
                    lidar_msg.data=True
                    motor_msg = Int32MultiArray()
                    motor_msg.data = pwm
                    #self.publisher_bool.publish(lidar_msg)
                    self.publisher.publish(motor_msg)
                    self.get_logger().info(f"Completed msg sent to obstacle_detection node")
                    return
            speeds = [abs(self.pid_controllers[i].compute(self.setpoints[i], self.encoder_values[i]) + self.encoder_values[i]) for i in range(4)]
            dirs=[[1,1,1,1],[1,1,-1,-1]]
            for i in range(4):
                left,right=0,self.sizes[i]-1
                while left<right-1:
                    mid=(left+right)//2
                    if speeds[i]<self.velocities[i][mid]:
                        left=mid
                    else:
                        right=mid
                ans=left if self.velocities[i][left]+self.velocities[i][right]<2*speeds[i] else right
                pwm[i]=(self.sizes[i]-ans)*(dirs[0][i] if self.rotated else dirs[1][i])
            # Publish motor speeds
            motor_msg = Int32MultiArray()
            motor_msg.data = pwm
            self.publisher.publish(motor_msg)
                #self.get_logger().info(f"Published motor value {self.encoder_values}")

    def update_pid_gains(self):
        """ Dynamically update PID gains from ROS2 parameters. """
        new_kp = self.get_parameter('kp').value
        new_ki = self.get_parameter('ki').value
        new_kd = self.get_parameter('kd').value

        for pid in self.pid_controllers:
            pid.update_gains(new_kp, new_ki, new_kd)

        self.get_logger().info(f"Updated PID Gains -> Kp: {new_kp}, Ki: {new_ki}, Kd: {new_kd}")


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

