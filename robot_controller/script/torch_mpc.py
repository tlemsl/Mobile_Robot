import casadi as ca
import numpy as np
import torch
import torch.nn as nn

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray


class RobotDynamicsModel(nn.Module):
    def __init__(self, input_size):
        super(RobotDynamicsModel, self).__init__()
        self.fc1 = nn.Linear(input_size, 2 * input_size)
        self.fc2 = nn.Linear(2 * input_size, 4 * input_size)
        self.fc3 = nn.Linear(4 * input_size, 6 * input_size)
        self.fc4 = nn.Linear(6 * input_size, 2 * input_size)
        self.output_layer = nn.Linear(2 * input_size, 3)  # Output size is |x| = 3

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = torch.relu(self.fc3(x))
        x = torch.relu(self.fc4(x))
        x = self.output_layer(x)
        return x

# Define the MPC controller class
class MPCController:
    def __init__(self, model, horizon=10, dt=0.1):
        self.model = model
        self.horizon = horizon  # Prediction horizon
        self.dt = dt  # Time step
        
        # Initialize state variables
        self.current_state = np.zeros(3)  # x, y, yaw
        
        # Set up the CasADi optimization problem
        self.opt_setup()
        
        # ROS node initialization
        rospy.init_node('mpc_controller')
        self.goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.update_goal)
        self.odom_subscriber = rospy.Subscriber('/visualizable_odom', Odometry, self.update_current_state)
        self.control_publisher = rospy.Publisher('/cmd_vel', Float32MultiArray, queue_size=1)

        # Default goal
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0
    
    def opt_setup(self):
        # CasADi symbolic variables
        x = ca.MX.sym('x')
        y = ca.MX.sym('y')
        yaw = ca.MX.sym('yaw')
        accel = ca.MX.sym('accel')
        steer = ca.MX.sym('steer')
        state = ca.vertcat(x, y, yaw)
        controls = ca.vertcat(accel, steer)

        # Objective function
        X = state
        cost = 0
        g = []

        # Loop over the prediction horizon
        for k in range(self.horizon):
            # Predict the next state
            X_next = self.model_predict(X, controls)
            X = X_next
            
            # Cost: distance to goal and control effort
            cost += ca.norm_2(X[:2] - ca.vertcat(self.goal_x, self.goal_y))**2 + ca.norm_2(X[2] - self.goal_yaw)**2
            cost += ca.norm_2(controls) ** 2

            # Constraints (vehicle dynamics)
            g.append(X)

        # Optimization problem
        opt_variables = ca.vertcat(state, controls)
        opt_params = ca.vertcat(self.goal_x, self.goal_y, self.goal_yaw)
        nlp = {'x': opt_variables, 'f': cost, 'g': ca.vertcat(*g)}

        # Solver setup
        opts = {'ipopt.print_level': 0, 'print_time': 0}
        self.solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

    def model_predict(self, state, controls):
        """
        Predict the next state using the trained model.
        """
        # Convert symbolic CasADi variables to numeric using numpy
        input_np = np.concatenate([ca.veccat(state), ca.veccat(controls)], axis=0)
        
        # Convert to PyTorch tensor
        input_tensor = torch.tensor(input_np, dtype=torch.float32).unsqueeze(0)

        # Predict the next state using the trained model
        with torch.no_grad():
            next_state = self.model(input_tensor).cpu().numpy().flatten()

        # Convert the next state back to CasADi MX format
        return ca.vertcat(next_state[0], next_state[1], next_state[2])


    def update_goal(self, msg):
        """
        Update the goal when a new goal is received.
        """
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_yaw = 2 * np.arctan2(msg.pose.orientation.z, msg.pose.orientation.w)

    def update_current_state(self, msg):
        """
        Update the current state based on odometry information.
        """
        self.current_state[0] = msg.pose.pose.position.x
        self.current_state[1] = msg.pose.pose.position.y
        self.current_state[2] = 2 * np.arctan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

    def compute_control(self):
        """
        Compute the optimal control input using MPC.
        """
        # Initial state
        x_init = ca.DM(self.current_state)
        
        # Solve the MPC problem
        sol = self.solver(x0=x_init)
        
        # Extract control inputs
        u = sol['x'][len(self.current_state):]
        accel = u[0].full()[0]
        steer = u[1].full()[0]

        # Publish control
        control_msg = Float32MultiArray(data=[accel, steer])
        self.control_publisher.publish(control_msg)

    def run(self):
        """
        Main loop to run the MPC controller.
        """
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Compute control
            self.compute_control()
            rate.sleep()

if __name__ == '__main__':
    # Load the trained PyTorch model
    h = 1
    model = RobotDynamicsModel(5 * h)
    model.load_state_dict(torch.load('/workspace/Documents/DynamicsLearning/RC/best_model_below_threshold.pth'))
    model.eval()

    # Create the MPC controller object
    mpc_controller = MPCController(model)

    # Run the MPC controller
    mpc_controller.run()
