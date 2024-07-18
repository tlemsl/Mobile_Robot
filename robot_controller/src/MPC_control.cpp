#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/tf.h>
#include <casadi/casadi.hpp>
#include <visualization_msgs/Marker.h>
#include <chrono>  // Include the chrono library


using namespace casadi;

class SimplePoseController {
public:
    SimplePoseController() {
        // Initialize the ROS node
        ros::NodeHandle nh;
        
        ros::AsyncSpinner spinner(4);
        // Subscriber to Gazebo model states
        pose_sub_ = nh.subscribe("/gazebo/model_states", 10, &SimplePoseController::poseCallback, this);
        
        // Subscriber to the goal pose
        goal_sub_ = nh.subscribe("/move_base_simple/goal", 10, &SimplePoseController::goalCallback, this);
        spinner.start();

        // Publisher to the robot's velocity command
        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        
        // Publisher for the current pose
        current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);

        // Publisher for the MPC trajectory
        trajectory_pub_ = nh.advertise<visualization_msgs::Marker>("/mpc_trajectory", 10);

        // Initialize robot and goal pose
        robot_pose_ = boost::none;
        goal_pose_ = boost::none;
        
        // Get the robot name from parameters or use default
        ros::NodeHandle pnh("~");
        pnh.param<std::string>("robot_name", robot_name_, "jackal");

        // MPC parameters
        dt_ = 0.5;
        N_ = 20;  // Prediction horizon
        Q_ = DM::diag(DM({10, 10, 1}));  // State weighting matrix
        R_ = DM::diag(DM({0.1, 0.1}));  // Control weighting matrix

        // Define the state and control variables
        x_ = MX::sym("x");
        y_ = MX::sym("y");
        theta_ = MX::sym("theta");
        v_ = MX::sym("v");
        omega_ = MX::sym("omega");
        state_ = vertcat(x_, y_, theta_);
        control_ = vertcat(v_, omega_);

        // Define the system dynamics for differential drive
        state_dot_ = vertcat(v_ * cos(theta_), v_ * sin(theta_), omega_);
        dynamics_ = Function("dynamics", {state_, control_}, {state_dot_});

        // Define the state and control boundaries
        x_min_ = DM({-5, -5, -M_PI});
        x_max_ = DM({10, 10, M_PI});
        v_min_ = -1;
        v_max_ = 1;
        omega_min_ = -M_PI/2;
        omega_max_ = M_PI/2;
        ros::waitForShutdown();
    }

    void poseCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        try {
            // Find the index of the robot in the ModelStates message
            auto it = std::find(msg->name.begin(), msg->name.end(), robot_name_);
            if (it != msg->name.end()) {
                size_t index = std::distance(msg->name.begin(), it);
                
                // Extract the robot's pose
                // ROS_INFO("Pose update!");
                robot_pose_ = msg->pose[index];
                
                // Publish the current pose
                publishCurrentPose();
            } else {
                ROS_WARN("Robot name not found in ModelStates");
            }
        } catch (const std::exception &e) {
            ROS_WARN("Exception in poseCallback: %s", e.what());
        }
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        goal_pose_ = *msg;
        ROS_INFO("New goal received: (%.2f, %.2f, %.2f)", msg->pose.position.x, msg->pose.position.y, getYaw(msg->pose.orientation));
        navigateToGoal();
    }

    double getYaw(const geometry_msgs::Quaternion& orientation) {
        tf::Quaternion quat;
        tf::quaternionMsgToTF(orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        return yaw;
    }

    void navigateToGoal() {
        // ros::Rate rate(10);  // 10 Hz
        while (ros::ok()) {
            if (robot_pose_ && goal_pose_) {
                // Get the current pose
                auto start = std::chrono::high_resolution_clock::now();

                double x = robot_pose_->position.x;
                double y = robot_pose_->position.y;
                double theta = getYaw(robot_pose_->orientation);
                
                // Get the goal pose
                double goal_x = goal_pose_->pose.position.x;
                double goal_y = goal_pose_->pose.position.y;
                double goal_theta = getYaw(goal_pose_->pose.orientation);
                
                DM x0 = DM::vertcat({x, y, theta});
                DM x_ref = DM::vertcat({goal_x, goal_y, goal_theta});
                
                // Solve the MPC problem
                std::pair<std::vector<double>, std::vector<DM>> result = solveMPC(x0, x_ref);
                std::vector<double> u_opt = result.first;
                std::vector<DM> X_pred = result.second;
                
                // Publish the velocity command
                publishVelocity(u_opt[0], u_opt[1]);
                
                // Publish the MPC trajectory
                publishTrajectory(X_pred);
                
                double dx = goal_x - x;
                double dy = goal_y - y;
                double dtheta = goal_theta - theta;
                double distance = sqrt(dx*dx + dy*dy + dtheta*dtheta);
                std::chrono::duration<double> duration = std::chrono::high_resolution_clock::now() - start;

                ROS_INFO("Goal distance: %.2f elepsed time %.2f", distance, duration.count());
                if (distance < 0.2) {
                    ROS_INFO("Reached the goal and aligned!");
                    publishVelocity(0, 0);
                    break;
                }
            } else {
                ROS_WARN("Waiting for robot pose and goal pose...");
            }
            // rate.sleep();
        }
    }

    std::pair<std::vector<double>, std::vector<DM>> solveMPC(const DM& x0, const DM& x_ref) {
        // Initialize optimization problem
        Opti opti;
        
        // Decision variables
        MX X = opti.variable(3, N_ + 1);  // state: [x, y, theta]
        MX U = opti.variable(2, N_);      // control: [v, omega]
        
        opti.subject_to(X(0, 0) == x0(0));
        opti.subject_to(X(1, 0) == x0(1));
        opti.subject_to(X(2, 0) == x0(2));
        
        // System dynamics constraints
        for (int k = 0; k < N_; ++k) {
            MX state_k = X(Slice(), k);
            MX control_k = U(Slice(), k);
            MX state_next = X(Slice(), k + 1);
            MX state_dot_k = dynamics_(std::vector<MX>{state_k, control_k})[0];
            opti.subject_to(state_next == state_k + dt_ * state_dot_k);
        }
        
        // Control and state constraints
        opti.subject_to(opti.bounded(x_min_, X, x_max_));
        opti.subject_to(opti.bounded(v_min_, U(Slice(0, 1), Slice()), v_max_));
        opti.subject_to(opti.bounded(omega_min_, U(Slice(1, 2), Slice()), omega_max_));
        
        // Objective function
        MX J = 0;
        for (int k = 0; k < N_; ++k) {
            MX state_error = X(Slice(), k) - x_ref;
            J += mtimes(state_error.T(), mtimes(Q_, state_error)) + mtimes(U(Slice(), k).T(), mtimes(R_, U(Slice(), k)));
        }
        opti.minimize(J);
        
        // Solver options
        Dict opts;
        opts["ipopt.print_level"] = 0;
        opts["print_time"] = 0;
        opts["ipopt.tol"] = 1e-6;
        opts["verbose"] = false;
        opti.solver("ipopt", opts);
        
        // Set initial state constraint
        opti.set_initial(X(Slice(), 0), x0);
        
        // Solve the optimization problem
        OptiSol sol = opti.solve();
        
        // Extract the optimal control input
        std::vector<double> u_opt = sol.value(U(Slice(), 0)).nonzeros();
        std::vector<DM> X_pred;
        for (int i = 0; i < N_ + 1; ++i) {
            X_pred.push_back(sol.value(X(Slice(), i)));
        }
        
        return std::make_pair(u_opt, X_pred);
    }

    void publishVelocity(double linear, double angular) {
        geometry_msgs::Twist twist;
        twist.linear.x = linear;
        twist.angular.z = angular;
        cmd_vel_pub_.publish(twist);
    }

    void publishCurrentPose() {
        if (robot_pose_) {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose = *robot_pose_;
            current_pose_pub_.publish(pose_stamped);
        }
    }

    void publishTrajectory(const std::vector<DM>& X_pred) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "mpc_trajectory";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.05;  // Line width
        marker.color.a = 1.0;  // Alpha
        marker.color.r = 0.0;  // Red
        marker.color.g = 0.0;  // Green
        marker.color.b = 1.0;  // Blue

        for (const auto& state : X_pred) {
            geometry_msgs::Point p;
            p.x = state(0).scalar();
            p.y = state(1).scalar();
            p.z = 0.0;
            marker.points.push_back(p);
        }

        trajectory_pub_.publish(marker);
    }

private:
    ros::Subscriber pose_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher current_pose_pub_;
    ros::Publisher trajectory_pub_;

    boost::optional<geometry_msgs::Pose> robot_pose_;
    boost::optional<geometry_msgs::PoseStamped> goal_pose_;
    
    std::string robot_name_;

    double dt_;
    int N_;
    DM Q_;
    DM R_;
    
    MX x_, y_, theta_, v_, omega_;
    MX state_, control_;
    MX state_dot_;
    Function dynamics_;

    DM x_min_, x_max_;
    double v_min_, v_max_;
    double omega_min_, omega_max_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_pose_controller");
    SimplePoseController controller;
    return 0;
}
