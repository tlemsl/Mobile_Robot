#!/usr/bin/env python

import rospy
import math
import casadi as ca
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class SimplePoseController:
    def __init__(self):
        # Initialize the node
        rospy.init_node('simple_pose_controller', anonymous=True)
        
        # Subscriber to Gazebo model states
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.pose_callback)
        rospy.Subscriber("/odom_pose", PoseStamped, self.gps_pose_callback)

        
        # Subscriber to the goal pose
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        rospy.Subscriber("/goal", PoseStamped, self.goal_callback)

        # Publisher to the robot's velocity command
        self.cmd_vel_pub = rospy.Publisher("/base_board/cmd", TwistStamped, queue_size=10)
        
        # Publisher for the current pose
        self.current_pose_pub = rospy.Publisher("/current_pose", PoseStamped, queue_size=10)

        # Publisher for the MPC trajectory
        self.trajectory_pub = rospy.Publisher("/mpc_trajectory", Marker, queue_size=10)

        # Initialize robot and goal pose
        self.robot_pose = None
        self.goal_pose = None
        
        # Robot name in Gazebo
        self.robot_name = rospy.get_param('~robot_name', 'robot')
        self.robot_name = "jackal"

        # MPC parameters
        self.dt = 0.1
        self.N = 10  # Prediction horizon
        self.Q = np.diag([10, 10, 5])  # State weighting matrix (no need to include v, delta is controlled directly)
        self.R = np.diag([0.1, 0.1])  # Control weighting matrix

        # Define the state and control variables
        self.x = ca.MX.sym('x')
        self.y = ca.MX.sym('y')
        self.theta = ca.MX.sym('theta')
        self.state = ca.vertcat(self.x, self.y, self.theta)
        
        # Control inputs: velocity (v) and steering angle (delta)
        self.v = ca.MX.sym('v')
        self.delta = ca.MX.sym('delta')
        self.control = ca.vertcat(self.v, self.delta)

        # Vehicle parameters
        self.L = 0.43  # Wheelbase
        self.L_r = 0.1  # Distance from rear axle to center of mass
        
        # Slip angle (beta)
        self.beta = ca.atan2(self.L_r * ca.tan(self.delta), self.L)

        # Define the system dynamics for the Bicycle model
        self.state_dot = ca.vertcat(
            self.v * ca.cos(self.theta + self.beta),
            self.v * ca.sin(self.theta + self.beta),
            self.v * ca.sin(self.beta) / self.L
        )
        self.dynamics = ca.Function('dynamics', [self.state, self.control], [self.state_dot])

        # Define the state and control boundaries
        self.x_min = np.array([-5, -5, -ca.pi])
        self.x_max = np.array([10, 10, ca.pi])
        self.v_min = -1
        self.v_max = 1
        self.delta_min = -ca.pi/4
        self.delta_max = ca.pi/4

    def pose_callback(self, msg):
        try:
            # Find the index of the robot in the ModelStates message
            index = msg.name.index(self.robot_name)
            
            # Extract the robot's pose
            self.robot_pose = msg.pose[index]
            # Publish the current pose
            self.publish_current_pose()
        except ValueError:
            rospy.logwarn("Robot name not found in ModelStates")
    
    def gps_pose_callback(self, msg):
        self.robot_pose = msg.pose

    def goal_callback(self, msg):
        self.goal_pose = msg
        rospy.loginfo("New goal received: (%.2f, %.2f, %.2f)", msg.pose.position.x, msg.pose.position.y, self.get_yaw(msg.pose.orientation))
        self.navigate_to_goal()

    def get_yaw(self, orientation):
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        return yaw

    def navigate_to_goal(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.robot_pose and self.goal_pose:
                # Get the current pose
                x = self.robot_pose.position.x
                y = self.robot_pose.position.y
                orientation_q = self.robot_pose.orientation
                theta = self.get_yaw(orientation_q)
                
                # Get the goal pose
                goal_x = self.goal_pose.pose.position.x
                goal_y = self.goal_pose.pose.position.y
                goal_theta = self.get_yaw(self.goal_pose.pose.orientation)
                
                x0 = np.array([x, y, theta])
                x_ref = np.array([goal_x, goal_y, goal_theta])
                
                # Solve the MPC problem
                u_opt, X_pred = self.solve_mpc(x0, x_ref)
                
                # Publish the velocity command
                self.publish_velocity(u_opt[0], u_opt[1])
                
                # Publish the MPC trajectory
                self.publish_trajectory(X_pred)
                
                dx = goal_x - x
                dy = goal_y - y
                dtheta = goal_theta - theta
                distance = math.sqrt(dx**2 + dy**2 + dtheta**2)
                rospy.loginfo(f"Goal distance {distance}")
                if distance < 0.2:
                    rospy.loginfo("Reached the goal and aligned!")
                    self.publish_velocity(0, 0)
                    break
            else:
                rospy.logwarn("Waiting for robot pose and goal pose...")
                
            rate.sleep()

    def solve_mpc(self, x0, x_ref):
        # Initialize optimization problem
        opti = ca.Opti()

        # Decision variables
        X = opti.variable(3, self.N + 1)  # state: [x, y, theta]
        U = opti.variable(2, self.N)      # control: [v, delta]

        opti.subject_to(X[:, 0] == x0)

        # System dynamics constraints
        for k in range(self.N):
            state_k = X[:, k]
            control_k = U[:, k]
            state_next = X[:, k + 1]
            state_dot_k = self.dynamics(state_k, control_k)
            opti.subject_to(state_next == state_k + self.dt * state_dot_k)

        # Control and state constraints
        # opti.subject_to(opti.bounded(self.x_min, X, self.x_max))
        opti.subject_to(opti.bounded(self.v_min, U[0, :], self.v_max))
        opti.subject_to(opti.bounded(self.delta_min, U[1, :], self.delta_max))

        # Objective function
        J = 0
        for k in range(self.N):
            state_error = X[:, k] - x_ref
            J += ca.mtimes(state_error.T, ca.mtimes(self.Q, state_error)) + ca.mtimes(U[:, k].T, ca.mtimes(self.R, U[:, k]))
        opti.minimize(J)

        # Solver options
        opts = {'ipopt.print_level': 0,  'print_time':0, 'ipopt.tol': 1e-6, 'verbose': False}
        opti.solver('ipopt', opts)
        
        # Set initial state constraint
        opti.set_initial(X[:, 0], x0)
        
        # Solve the optimization problem
        sol = opti.solve()
        
        # Extract the optimal control input
        u_opt = sol.value(U[:, 0])
        X_pred = sol.value(X)
        
        return u_opt, X_pred

    def publish_velocity(self, linear_velocity, steering_angle):
        twist = TwistStamped()
        twist.twist.linear.x = linear_velocity
        twist.twist.angular.z = steering_angle
        self.cmd_vel_pub.publish(twist)
    
    def publish_current_pose(self):
        if self.robot_pose:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = "odom"
            pose_stamped.pose = self.robot_pose
            self.current_pose_pub.publish(pose_stamped)
    
    def publish_trajectory(self, X_pred):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "mpc_trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Line width
        marker.color.a = 1.0  # Alpha
        marker.color.r = 0.0  # Red
        marker.color.g = 0.0  # Green
        marker.color.b = 1.0  # Blue
        for i in range(self.N + 1):
            p = Point()
            p.x = X_pred[0, i]
            p.y = X_pred[1, i]
            p.z = 0.0
            marker.points.append(p)
        
        self.trajectory_pub.publish(marker)

if __name__ == '__main__':
    try:
        controller = SimplePoseController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
