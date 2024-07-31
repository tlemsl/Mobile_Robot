#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <casadi/casadi.hpp>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>
#include <map>

#include <algorithm>
#include <chrono>  // Include the chrono library

#include <Eigen/Core>

using namespace casadi;

class MPCController {
public:
    MPCController() {
        // Initialize the ROS node
        ros::NodeHandle nh;
        ros::AsyncSpinner spinner(4);

        // Get the use_sim and path_tracking parameters
        ros::NodeHandle pnh("~");
        bool use_sim;
        bool path_tracking;
        pnh.param("use_sim", use_sim, true);
        pnh.param("path_tracking", path_tracking, false);
        pnh.param("path_tracking_mode", path_tracking_mode_, 0);
        // Conditional subscription based on use_sim
        if (use_sim) {
            ROS_INFO("Start controller under sim env");
            pose_sub_ = nh.subscribe("/gazebo/model_states", 10, &MPCController::poseCallbackSim, this);
        } else {
            ROS_INFO("Start controller under real env");
            pose_sub_ = nh.subscribe("/robot_pose", 10, &MPCController::poseCallbackReal, this);
        }

        // Subscriber to the goal pose or global path based on path_tracking
        if (path_tracking) {
            ROS_INFO("Path tracking mode %d", path_tracking_mode_);
            path_sub_ = nh.subscribe("/global_path", 10, &MPCController::pathCallback, this);
        } else {
            ROS_INFO("Pose tracking mode");
            goal_sub_ = nh.subscribe("/move_base_simple/goal", 1, &MPCController::goalCallback, this);
        }
        
        spinner.start();

        // Publisher to the robot's velocity command
        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        
        // Publisher for the current pose
        current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1);

        // Publisher for the MPC trajectory
        trajectory_pub_ = nh.advertise<visualization_msgs::Marker>("/mpc_trajectory", 1);

        debug_pub_ = nh.advertise<std_msgs::Float32MultiArray>("/tracking_error", 1);

        // Initialize robot and goal pose
        robot_pose_ = boost::none;
        goal_pose_ = boost::none;
        
        // Get the robot name from parameters or use default
        pnh.param<std::string>("robot_name", robot_name_, "jackal");
        pnh.param<std::string>("logfile_path", logfile_path_, "/workspace");

        // MPC parameters
        pnh.param("dt", dt_, 0.5);
        pnh.param("N", N_, 20);
        
        double xy_cost, yaw_cost;
        pnh.param("xy_cost", xy_cost, 10.0);
        pnh.param("yaw_cost", yaw_cost, 1.0);

        std::vector<double> Q_vec = {xy_cost, xy_cost, yaw_cost};
        Q_ = DM::diag(DM(Q_vec));
        
        double v_cost, omega_cost;
        pnh.param("v_cost_", v_cost, 0.1);
        pnh.param("omega_cost_", omega_cost, 0.1);
        std::vector<double> R_vec = {v_cost, omega_cost};
        R_ = DM::diag(DM(R_vec));

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
        std::vector<double> x_min_vec = {-1000, -1000, -2*M_PI};
        std::vector<double> x_max_vec = {1000, 1000, 2*M_PI};
        // TODO(Minjong) ROS array parameters?
        // pnh.param("x_min", x_min_vec, {-5, -5, -M_PI});
        // pnh.getParam("x_max", x_max_vec);
        // ROS_INFO_STREAM(x_min_vec);
        x_min_ = DM(x_min_vec);
        x_max_ = DM(x_max_vec);

        pnh.param("v_min", v_min_, -1.0);
        pnh.param("v_max", v_max_, 1.0);
        pnh.param("omega_min", omega_min_, -M_PI/2);    
        pnh.param("omega_max", omega_max_, M_PI/2);

        pnh.param("goal_dist_th", goal_dist_th_, 0.2);
        pnh.param("update_dist_th", update_dist_th_, 0.1);

        std::string full_filename = logfile_path_ + "data" + ".csv";
        file_ = std::ofstream(full_filename);

        if (!file_.is_open()) {
            std::cerr << "Error opening file_: " << full_filename << std::endl;
            return;
        }
        // Write header
        file_ << "timestamp,dx,dy,dtheta,v,w\n";
        K_ << 0.2546, -0.5440,
              -1.2602, -8.0162,
              0,0;
        ros::waitForShutdown();
    }

    MX pi2pi(MX angle) {
        return fmod(angle + M_PI, 2 * M_PI) - M_PI;
    }

    double pi2pi(double angle){
        return fmod(angle + M_PI, 2 * M_PI) - M_PI;
    }

    void poseCallbackSim(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        try {
            // Find the index of the robot in the ModelStates message
            auto it = std::find(msg->name.begin(), msg->name.end(), robot_name_);
            if (it != msg->name.end()) {
                size_t index = std::distance(msg->name.begin(), it);
                
                // Extract the robot's pose
                robot_pose_ = msg->pose[index];
                
                // Publish the current pose
                publishCurrentPose();
            } else {
                ROS_WARN("Robot name not found in ModelStates");
            }
        } catch (const std::exception &e) {
            ROS_WARN("Exception in poseCallbackSim: %s", e.what());
        }
    }

    void poseCallbackReal(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        try {
            // Extract the robot's pose
            robot_pose_ = msg->pose;
            
            // Publish the current pose
            publishCurrentPose();
        } catch (const std::exception &e) {
            ROS_WARN("Exception in poseCallbackReal: %s", e.what());
        }
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        goal_pose_ = *msg;
        ROS_INFO("New goal received: (%.2f, %.2f, %.2f)", msg->pose.position.x, msg->pose.position.y, getYaw(msg->pose.orientation));
        navigateToGoal();
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        // Process the path waypoints
        path_ = *msg;
        ROS_INFO("New path received with %zu waypoints.", path_.poses.size());
        if (path_tracking_mode_ == 0){
            navigateToPath();
        }
        else {
            navigateToPath2();
        }
        
    }

    double getYaw(const geometry_msgs::Quaternion& orientation) {
        tf::Quaternion quat;
        tf::quaternionMsgToTF(orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        return pi2pi(yaw);
    }

    void navigateToGoal() {
        while (ros::ok()) {
            if (robot_pose_ && goal_pose_) {
                auto start = std::chrono::high_resolution_clock::now();

                double x = robot_pose_->position.x;
                double y = robot_pose_->position.y;
                double theta = getYaw(robot_pose_->orientation);
                
                double goal_x = goal_pose_->pose.position.x;
                double goal_y = goal_pose_->pose.position.y;
                double goal_theta = getYaw(goal_pose_->pose.orientation);
                
                DM x0 = DM::vertcat({x, y, theta});
                DM x_ref = DM::vertcat({goal_x, goal_y, goal_theta});
                
                std::pair<std::vector<double>, std::vector<DM>> result = solveMPC(x0, x_ref);
                std::vector<double> u_opt = result.first;
                std::vector<DM> X_pred = result.second;
                
                
                publishTrajectory(X_pred);
                
                double dx = goal_x - x;
                double dy = goal_y - y;
                double dtheta = goal_theta - theta;
                double distance = sqrt(dx*dx + dy*dy);
                std::chrono::duration<double> duration = std::chrono::high_resolution_clock::now() - start;

                dt_ = duration.count();
                ROS_INFO("Goal distance: %.2f D theta: %.2f elapsed time %.2f", distance, dtheta, duration.count());
                
                publishErrorInfo(dx, dy, dtheta);
                publishVelocity(u_opt[0], u_opt[1], dx, dy, dtheta);
                error_dynamics_[ros::Time::now()] = {dx, dy, dtheta, u_opt[0], u_opt[1]};
                if (distance < goal_dist_th_ && std::abs(dtheta) < 0.01 && u_opt[0] < 0.1 && u_opt[1] < 0.1) {
                    ROS_INFO("Reached the goal and aligned!");
                    publishVelocity(0, 0);
                    writeCSV();
                    break;
                }
            } else {
                ROS_WARN("Waiting for robot pose and goal pose...");
            }
        }
    }

    void navigateToPath() {
        int nearest_index = 0;
        while (ros::ok()) {
            if (robot_pose_ && !path_.poses.empty()) {
                double x = robot_pose_->position.x;
                double y = robot_pose_->position.y;
                double theta = getYaw(robot_pose_->orientation);

                size_t last_idx = static_cast<int>(path_.poses.size() - 1);
                double goal_x = path_.poses[last_idx].pose.position.x;
                double goal_y = path_.poses[last_idx].pose.position.y;
                double goal_theta = getYaw(path_.poses[last_idx].pose.orientation);

                nearest_index = findNearestIndex(nearest_index);

                auto start = std::chrono::high_resolution_clock::now();

                DM x0 = DM::vertcat({x, y, theta});

                std::pair<std::vector<double>, std::vector<DM>> result = solveMPCPath(x0, nearest_index);
                std::vector<double> u_opt = result.first;
                std::vector<DM> X_pred = result.second;

                publishTrajectory(X_pred);
                publishTargetTrajectory(nearest_index);
                
                double dx = goal_x - x;
                double dy = goal_y - y;
                double dtheta = pi2pi(goal_theta - theta);
                double distance = sqrt(dx*dx + dy*dy);
                publishVelocity(u_opt[0], u_opt[1], dx, dy, dtheta);

                std::chrono::duration<double> duration = std::chrono::high_resolution_clock::now() - start;
                
                dt_ = duration.count();
                ROS_INFO("Target idx: %d Goal distance: %.2f elapsed time %.2f",nearest_index, distance, duration.count());

                publishErrorInfo(dx, dy, dtheta);
                error_dynamics_[ros::Time::now()] = {dx,dy, dtheta, u_opt[0], u_opt[1]};

                if (distance < goal_dist_th_&& dtheta < 0.05 && u_opt[0] < 0.1 && u_opt[1] < 0.1) {
                    ROS_INFO("Reached the goal and aligned!");
                    publishVelocity(0, 0);
                    writeCSV();
                    break;
                }
            } else {
                ROS_WARN("Waiting for robot pose and path...");
            }
        }
    }

    void navigateToPath2() {
        int target_index = 0;

        while (ros::ok()) {
            if (robot_pose_ && !path_.poses.empty()) {
                auto start = std::chrono::high_resolution_clock::now();
                target_index = updateTargetIndex(target_index);

                double x = robot_pose_->position.x;
                double y = robot_pose_->position.y;
                double theta = getYaw(robot_pose_->orientation);
                
                double goal_x = path_.poses[target_index].pose.position.x;
                double goal_y = path_.poses[target_index].pose.position.y;
                double goal_theta = getYaw(path_.poses[target_index].pose.orientation);

                double dx = goal_x - x;
                double dy = goal_y - y;
                double dtheta = pi2pi(goal_theta - theta);
                double distance = sqrt(dx*dx + dy*dy);

                DM x0 = DM::vertcat({x, y, theta});
                DM x_ref = DM::vertcat({goal_x, goal_y, goal_theta});
                
                std::pair<std::vector<double>, std::vector<DM>> result = solveMPC(x0, x_ref);
                std::vector<double> u_opt = result.first;
                std::vector<DM> X_pred = result.second;
                
                publishVelocity(u_opt[0], u_opt[1], dx, dy, dtheta);
                publishTrajectory(X_pred);
                publishTargetTrajectory(target_index);
                publishErrorInfo(dx, dy, dtheta);
                error_dynamics_[ros::Time::now()] = {dx, dy, dtheta, u_opt[0], u_opt[1]};

                int last_idx = static_cast<int>(path_.poses.size() - 1);
                goal_x = path_.poses[last_idx].pose.position.x;
                goal_y = path_.poses[last_idx].pose.position.y;
                goal_theta = getYaw(path_.poses[last_idx].pose.orientation);

                dx = goal_x - x;
                dy = goal_y - y;
                dtheta = pi2pi(goal_theta - theta);
                distance = sqrt(dx*dx + dy*dy + dtheta*dtheta);
                std::chrono::duration<double> duration = std::chrono::high_resolution_clock::now() - start;
                
                dt_ = duration.count();
                ROS_INFO("Target idx: %d Goal distance: %.2f elapsed time %.2f",target_index, distance, duration.count());
                
                if (distance < goal_dist_th_ && u_opt[0] < 0.1 && u_opt[1] < 0.1 && target_index == last_idx) {
                    ROS_INFO("Reached the goal and aligned!");
                    publishVelocity(0, 0);
                    writeCSV();
                    break;
                }
            } else {
                ROS_WARN("Waiting for robot pose and goal pose...");
            }
        }
    }
    std::pair<std::vector<double>, std::vector<DM>> solveMPC(const DM& x0, const DM& x_ref) {
        // Initialize optimization problem
        Opti opti;
        
        // Decision variables
        MX X = opti.variable(3, N_ + 1);  // state: [x, y, theta]
        MX U = opti.variable(2, N_);      // control: [v, omega]
        
        opti.subject_to(X(Slice(), 0) == x0);
        
        // System dynamics constraints w forward euler methods
        // for (int k = 0; k < N_; ++k) {
        //     MX state_k = X(Slice(), k);
        //     MX control_k = U(Slice(), k);
        //     MX state_next = X(Slice(), k + 1);
        //     MX state_dot_k = dynamics_(std::vector<MX>{state_k, control_k})[0];
        //     opti.subject_to(state_next == state_k + dt_ * state_dot_k);
        // }

        // System dynamics constraints w RK4
        for (int k = 0; k < N_; ++k) {
            MX state_k = X(Slice(), k);
            MX control_k = U(Slice(), k);
            MX state_next = X(Slice(), k + 1);
            
            // RK4 method calculations
            MX k1 = dynamics_(std::vector<MX>{state_k, control_k})[0];
            MX k2 = dynamics_(std::vector<MX>{state_k + dt_/2 * k1, control_k})[0];
            MX k3 = dynamics_(std::vector<MX>{state_k + dt_/2 * k2, control_k})[0];
            MX k4 = dynamics_(std::vector<MX>{state_k + dt_ * k3, control_k})[0];
            
            MX state_next_rk4 = state_k + (dt_/6) * (k1 + 2*k2 + 2*k3 + k4);
            
            opti.subject_to(state_next == state_next_rk4);
        }
        
        // Control and state constraints
        // opti.subject_to(opti.bounded(x_min_, X, x_max_));
        opti.subject_to(opti.bounded(v_min_, U(0, Slice()), v_max_));
        opti.subject_to(opti.bounded(omega_min_, U(1, Slice()), omega_max_));
        
        // Objective function
        MX J = 0;
        for (int k = 0; k < N_; ++k) {
            MX state_error = X(Slice(), k) - x_ref;
            state_error(2) = pi2pi(state_error(2));
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

    std::pair<std::vector<double>, std::vector<DM>> solveMPCPath(const DM& x0, int nearest_index) {
        Opti opti;
        MX X = opti.variable(3, N_ + 1); // State trajectory
        MX U = opti.variable(2, N_);     // Control trajectory

        opti.subject_to(X(Slice(), 0) == x0);

        // System dynamics constraints w forward euler methods
        // for (int k = 0; k < N_; ++k) {
        //     MX state_k = X(Slice(), k);
        //     MX control_k = U(Slice(), k);
        //     MX state_next = X(Slice(), k + 1);
        //     MX state_dot_k = dynamics_(std::vector<MX>{state_k, control_k})[0];
        //     opti.subject_to(state_next == state_k + dt_ * state_dot_k);
        // }

        // System dynamics constraints w RK4
        for (int k = 0; k < N_; ++k) {
            MX state_k = X(Slice(), k);
            MX control_k = U(Slice(), k);
            MX state_next = X(Slice(), k + 1);
            
            // RK4 method calculations
            MX k1 = dynamics_(std::vector<MX>{state_k, control_k})[0];
            MX k2 = dynamics_(std::vector<MX>{state_k + dt_/2 * k1, control_k})[0];
            MX k3 = dynamics_(std::vector<MX>{state_k + dt_/2 * k2, control_k})[0];
            MX k4 = dynamics_(std::vector<MX>{state_k + dt_ * k3, control_k})[0];
            
            MX state_next_rk4 = state_k + (dt_/6) * (k1 + 2*k2 + 2*k3 + k4);
            
            opti.subject_to(state_next == state_next_rk4);
        }

        // Control and state constraints
        // opti.subject_to(opti.bounded(x_min_, X, x_max_));
        opti.subject_to(opti.bounded(v_min_, U(0, Slice()), v_max_));
        opti.subject_to(opti.bounded(omega_min_, U(1, Slice()), omega_max_));

        // Set initial state constraint
        opti.set_initial(X(Slice(), 0), x0);
        
        // Objective function
        MX J = 0;
        size_t path_length = path_.poses.size();
 
        for (int k = 0; k < N_; ++k) {
            MX state = X(Slice(), k);

            // Make sure the nearest index + k does not exceed the path length
            int index = std::min(nearest_index + k, static_cast<int>(path_length - 1));

            // Get the reference point from the path
            double ref_x = path_.poses[index].pose.position.x;
            double ref_y = path_.poses[index].pose.position.y;
            double ref_theta = getYaw(path_.poses[index].pose.orientation);

            // Calculate the error between the state and the reference
            MX state_error = state - MX::vertcat({ref_x, ref_y, ref_theta});
            state_error(2) = pi2pi(state_error(2));
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

    int findNearestIndex(int pre_idx) {
        double min_distance = std::numeric_limits<double>::infinity();
        int nearest_index = 0;
        const double state_x = robot_pose_->position.x;
        const double state_y = robot_pose_->position.y;

        for (size_t i = pre_idx; i < path_.poses.size(); ++i) {
            double path_x = path_.poses[i].pose.position.x;
            double path_y = path_.poses[i].pose.position.y;

            double dx = state_x - path_x;
            double dy = state_y - path_y;
            double distance = std::sqrt(dx * dx + dy * dy);

            if (distance < min_distance) {
                min_distance = distance;
                nearest_index = i;
            }
        }

        return nearest_index;
    }

    int updateTargetIndex(int index) {
        const double state_x = robot_pose_->position.x;
        const double state_y = robot_pose_->position.y;
        int path_size = static_cast<int>(path_.poses.size());
        for(int i = index; i<path_size; i++){
            const double path_x = path_.poses[i].pose.position.x;
            const double path_y = path_.poses[i].pose.position.y;
            
            const double dx = state_x - path_x;
            const double dy = state_y - path_y;
            const double distance = std::sqrt(dx * dx + dy * dy);
            if(distance > update_dist_th_) {
               return i;
            }
        }
        
            
        return path_size - 1;
    }

    void publishVelocity(double linear, double angular, double dx = 0,  double dy = 0, double dtheta = 0) {
        Eigen::Vector3d error;
        error(0) = dx;
        error(1) = dy;
        error(2) = dtheta;

        Eigen::Vector2d auxilary_input = K_*error;

        geometry_msgs::Twist twist;
        // twist.linear.x = auxilary_input[0];
        // twist.angular.z = auxilary_input[1];
        twist.linear.x = linear;
        twist.angular.z = angular;


        twist.linear.x = std::min(v_max_, std::max(v_min_, twist.linear.x));
        twist.angular.z = std::min(omega_max_, std::max(omega_min_, twist.angular.z));
        

        ROS_INFO("MPC input v: %.2f w: %.2f\t Auxilary input v: %.2f w: %.2f", linear, angular, auxilary_input[0], auxilary_input[1]);
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

    void publishTargetTrajectory(int index) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "mpc_target_trajectory";
        marker.id = 1;
        
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1;  // Line width
        marker.color.a = 1.0;   // Alpha
        marker.color.r = 1.0;   // Red
        marker.color.g = 0.0;   // Green
        marker.color.b = 0.0;   // Blue
        

        if(path_tracking_mode_ == 0){
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            int path_length = static_cast<int>(path_.poses.size());
            int last_index = std::min(path_length, index + N_);

            for (int i=index; i< last_index; ++i) {
                geometry_msgs::Point p;
                p.x = path_.poses[i].pose.position.x;
                p.y = path_.poses[i].pose.position.y;
                p.z = 0.0;
                marker.points.push_back(p);
            }
        }
        else {
            marker.type = visualization_msgs::Marker::ARROW;
            marker.scale.x = 0.3;  
            marker.scale.y = 0.1;  
            marker.scale.z = 0.1;  // Line width
            marker.pose = path_.poses[index].pose;          
        }
        trajectory_pub_.publish(marker);
    }
    void publishErrorInfo(double dx, double dy, double dtheta) {
        std_msgs::Float32MultiArray msg;
        msg.data.push_back(dx);
        msg.data.push_back(dy);
        msg.data.push_back(dtheta);
        debug_pub_.publish(msg);
    }

    std::string toCSVString(const ros::Time& time) {
        return std::to_string(time.sec) + "." + std::to_string(time.nsec);
    }

    void writeCSV() {
        



        for (const auto& entry : error_dynamics_) {
            const ros::Time& timestamp = entry.first;
            const std::vector<double>& values = entry.second;

            if (values.size() == 5) {
                file_ << toCSVString(timestamp) << ","
                    << values[0] << ","
                    << values[1] << ","
                    << values[2] << ","
                    << values[3] << ","
                    << values[4] << "\n";
            } else {
                std::cerr << "Unexpected vector size for timestamp " << toCSVString(timestamp) << std::endl;
            }
        }

        file_.flush();
        error_dynamics_.clear();
    }
    
private:
    ros::Subscriber pose_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber path_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher current_pose_pub_;
    ros::Publisher trajectory_pub_;
    ros::Publisher debug_pub_;

    boost::optional<geometry_msgs::Pose> robot_pose_;
    boost::optional<geometry_msgs::PoseStamped> goal_pose_;
    nav_msgs::Path path_;

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

    double goal_dist_th_;
    double update_dist_th_;

    int path_tracking_mode_ = 0; // 0: horizon cost, 1: target cost
    
    std::string logfile_path_;
    int log_number = 0;
    std::map<ros::Time, std::vector<double>> error_dynamics_;

    Eigen::Matrix<double, 2,3> K_;
    std::ofstream file_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_pose_controller");
    MPCController controller;
    return 0;
}
