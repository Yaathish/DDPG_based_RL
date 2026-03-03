# Importing ROS2 Python client library
import rclpy
# Importing Node class to create ROS2 nodes
from rclpy.node import Node
# Importing message types for inter-node communication
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from sensor_msgs.msg import LaserScan
# Importing numpy for matrix operations
import numpy as np
# Importing math functions
import math

# Defining the QLearningNode class for obstacle-aware navigation using Q-learning
class QLearningNode(Node):
    # Initializing the QLearningNode
    def __init__(self):
        # Initializing the node with the name 'q_learning_node'
        super().__init__('q_learning_node')
        
        # Creating a subscriber to receive obstacle positions (angle, distance) from lidar
        self.lidar_sub = self.create_subscription(
            Float32MultiArray, 'lidar_val', self.lidar_callback, 10)

        # Creating a publisher to send planned positions to the robot
        self.publisher = self.create_publisher(
            Int32MultiArray, 'path_data', 10)

        # Defining the grid size (7x7 = 49 cells); each cell ~25cm
        self.grid_size = 7

        # Initializing the Q-table with action values and obstacle flags per cell
        self.q_table = np.zeros((self.grid_size, self.grid_size), 
                                dtype=[('action', 'float64', 4), ('obstacle', 'bool')])

        # Defining learning parameters
        self.alpha = 0.1                 # Learning rate
        self.gamma = 0.9                 # Discount factor
        self.epsilon = 1.0               # Exploration rate
        self.epsilon_min = 0.01          # Minimum exploration rate
        self.epsilon_decay = 0.99        # Epsilon decay per episode

        # Initializing episode control variables
        self.timeout_count = 25          # Max steps before timeout
        self.action_no = 0               # Number of actions taken in an episode
        self.robot_position = (0, 0)     # Current position of the robot
        self.goal_position = (7, 7)      # Goal position in the grid
        self.action = None               # Last action taken
        self.episode_path = []           # Path of actions taken in current episode
        self.returning = False           # Flag indicating robot is returning to start
        self.new_position = [0, 0]       # Next position based on selected action
        self.max_episodes = 200          # Total number of episodes to train
        self.current_episode = 0         # Current episode number

        # Logging node initialization
        self.get_logger().info("Q-Learning node Started")

    # Defining the callback for lidar data reception
    def lidar_callback(self, msg):
        # Updating robot position to the new position from the previous action
        self.robot_position = self.new_position

        # Storing lidar obstacle data
        lidar_data = msg.data

        # Logging debug message
        self.get_logger().info("Hi2")

        # Processing the obstacle data and updating the Q-table
        self.process_lidar_data(lidar_data)

        # Taking the next action (exploration or exploitation)
        self.take_action()

    # Defining function to convert lidar data to obstacle flags in grid
    def process_lidar_data(self, lidar_data):
        for i in range(0, len(lidar_data), 2):
            [angle, distance] = [lidar_data[i], lidar_data[i + 1]]
            if distance < 0.5:  # Only obstacles within 50cm
                obs_x = self.robot_position[0] + math.ceil((distance * np.cos(np.radians(angle))) * 4)
                obs_y = self.robot_position[1] - math.ceil((distance * np.sin(np.radians(angle))) * 4)
                if 0 <= obs_x < self.grid_size and 0 <= obs_y < self.grid_size:
                    self.q_table[obs_x, obs_y]['obstacle'] = True

    # Defining action selection logic (epsilon-greedy with obstacle check)
    def choose_action(self):
        valid_action_found = False
        while not valid_action_found:
            if np.random.rand() < self.epsilon:
                action = np.random.choice(4)  # Explore randomly
            else:
                action = np.argmax(self.q_table[self.robot_position]['action'])  # Exploit best action
            self.new_position = self.get_new_position(self.robot_position, action)
            if self.is_valid_position(self.new_position):
                valid_action_found = True
                self.get_logger().info("Valid!!")
        return action

    # Defining the function to take an action or replay the path back
    def take_action(self):
        if not self.returning:
            self.action = self.choose_action()
            self.episode_path.append(self.action)
            self.action_no += 1
        else:
            if self.episode_path:
                self.action = self.episode_path.pop()
            else:
                self.reset_episode()
                return

        # Publishing the new position as the robot's next move
        msg = Int32MultiArray() if not self.returning else 0
        msg.data = self.new_position
        self.publisher.publish(msg)

        # Logging the move
        self.get_logger().info(f"{msg.data}")

        # Updating Q-table if still in forward path
        if not self.returning:
            reward = self.get_reward()
            self.update_q_table(reward)

            # Check if reached goal or timeout
            if self.robot_position == self.goal_position or self.action_no == self.timeout_count:
                self.returning = True  # Begin path replay phase

    # Defining utility to calculate new grid position based on action
    def get_new_position(self, position, action):
        moves = [(1, 0), (0, 1), (-1, 0), (0, -1)]  # Right, Up, Left, Down
        dx, dy = moves[action]
        return [position[0] + dx, position[1] + dy]

    # Defining check to ensure the position is within grid and not blocked
    def is_valid_position(self, position):
        x, y = position
        return 0 <= x < self.grid_size and 0 <= y < self.grid_size and not self.q_table[x, y]['obstacle']

    # Defining reward function for Q-learning
    def get_reward(self):
        if self.new_position == self.goal_position:
            return 100
        if self.q_table[tuple(self.new_position)]['obstacle']:
            return -10
        return -1

    # Defining function to update the Q-table using Bellman equation
    def update_q_table(self, reward):
        self.q_table[self.robot_position[0], self.robot_position[1]]['action'][self.action] += self.alpha * (
            reward + self.gamma * np.max(self.q_table[self.new_position]['action']) -
            self.q_table[self.robot_position[0], self.robot_position[1]]['action'][self.action]
        )
        self.get_logger().info(f"{self.q_table}")

    # Defining function to reset episode after reaching goal or timeout
    def reset_episode(self):
        if self.current_episode >= self.max_episodes:
            self.get_logger().info("Training Completed after 200 episodes.")
            rclpy.shutdown()
            return
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay
        self.current_episode += 1
        self.returning = False
        self.episode_path = []
        self.action_no = 0
        self.get_logger().info(f"New Episode {self.current_episode} Started")

# Defining the main function to run the node
def main(args=None):
    # Initializing ROS2 system
    rclpy.init(args=args)
    # Creating and spinning the QLearningNode
    node = QLearningNode()
    rclpy.spin(node)
    # Cleaning up
    node.destroy_node()
    rclpy.shutdown()

# Starting point of execution
if __name__ == '__main__':
    main()
