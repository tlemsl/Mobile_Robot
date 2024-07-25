#!/usr/bin/env python

import rospy
import argparse
import math
import time
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion

current_pose = None

def pose_callback(msg):
    global current_pose
    current_pose = msg

def transform_point(x, y, yaw, start_pose):
    # Extract position and orientation (yaw) from the start_pose
    start_x = start_pose.pose.position.x
    start_y = start_pose.pose.position.y
    orientation = start_pose.pose.orientation
    _, _, start_yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    
    # Rotate and translate the point
    x_rot = x * np.cos(start_yaw) - y * np.sin(start_yaw)
    y_rot = x * np.sin(start_yaw) + y * np.cos(start_yaw)
    x_trans = x_rot + start_x
    y_trans = y_rot + start_y

    # Adjust the yaw to be relative to the start pose's yaw
    new_yaw = yaw + start_yaw
    
    return x_trans, y_trans, new_yaw

def generate_sin_path(length, fragment_size, start_pose):
    path = Path()
    path.header.frame_id = "map"
    num_points = int(length / fragment_size)
    for i in range(num_points):
        x = i * fragment_size
        y = 0.15* length * np.cos(x * (2 * np.pi / length)) - 0.15 * length
        yaw = np.arctan2( - 0.15* length* np.sin(x * (2 * np.pi / length)) * (2 * np.pi / length), 1)
        x_trans, y_trans, new_yaw = transform_point(x, y, yaw, start_pose)
        quat = quaternion_from_euler(0, 0, new_yaw)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x_trans
        pose.pose.position.y = y_trans
        pose.pose.orientation = Quaternion(*quat)
        path.poses.append(pose)
    return path

def generate_straight_path(length, fragment_size, start_pose):
    path = Path()
    path.header.frame_id = "map"
    num_points = int(length / fragment_size)
    for i in range(num_points):
        x = i * fragment_size
        y = 0
        yaw = 0
        x_trans, y_trans, new_yaw = transform_point(x, y, yaw, start_pose)
        quat = quaternion_from_euler(0, 0, new_yaw)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x_trans
        pose.pose.position.y = y_trans
        pose.pose.orientation = Quaternion(*quat)
        path.poses.append(pose)
    return path

def generate_square_path(side_length, fragment_size, start_pose):
    path = Path()
    path.header.frame_id = "map"
    num_points_per_side = int(side_length / fragment_size)

    for i in range(num_points_per_side):
        x = i * fragment_size
        y = 0
        yaw = 0
        x_trans, y_trans, new_yaw = transform_point(x, y, yaw, start_pose)
        quat = quaternion_from_euler(0, 0, new_yaw)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x_trans
        pose.pose.position.y = y_trans
        pose.pose.orientation = Quaternion(*quat)
        path.poses.append(pose)

    for i in range(num_points_per_side):
        x = side_length
        y = i * fragment_size
        yaw = np.pi / 2
        x_trans, y_trans, new_yaw = transform_point(x, y, yaw, start_pose)
        quat = quaternion_from_euler(0, 0, new_yaw)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x_trans
        pose.pose.position.y = y_trans
        pose.pose.orientation = Quaternion(*quat)
        path.poses.append(pose)

    for i in range(num_points_per_side):
        x = side_length - i * fragment_size
        y = side_length
        yaw = np.pi
        x_trans, y_trans, new_yaw = transform_point(x, y, yaw, start_pose)
        quat = quaternion_from_euler(0, 0, new_yaw)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x_trans
        pose.pose.position.y = y_trans
        pose.pose.orientation = Quaternion(*quat)
        path.poses.append(pose)

    for i in range(num_points_per_side):
        x = 0
        y = side_length - i * fragment_size
        yaw = -np.pi / 2
        x_trans, y_trans, new_yaw = transform_point(x, y, yaw, start_pose)
        quat = quaternion_from_euler(0, 0, new_yaw)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x_trans
        pose.pose.position.y = y_trans
        pose.pose.orientation = Quaternion(*quat)
        path.poses.append(pose)

    return path

def main():
    global current_pose

    parser = argparse.ArgumentParser(description="Path Publisher Node")
    parser.add_argument('--shape', choices=['sin', 'line', 'square'], required=True, help="Shape of the path to generate")
    parser.add_argument('--length', type=float, default=10.0, help="Total length of the path or side length for square")
    parser.add_argument('--fragment_size', type=float, default=0.5, help="Size of each path fragment")

    args = parser.parse_args()

    rospy.init_node('path_publisher', anonymous=True)

    rospy.Subscriber("/current_pose", PoseStamped, pose_callback)
    path_pub = rospy.Publisher('/global_path', Path, queue_size=1)

    # Wait for the current_pose to be received
    while current_pose is None:
        rospy.sleep(0.1)

    if args.shape == 'sin':
        path = generate_sin_path(args.length, args.fragment_size, current_pose)
    elif args.shape == 'line':
        path = generate_straight_path(args.length, args.fragment_size, current_pose)
    elif args.shape == 'square':
        path = generate_square_path(args.length, args.fragment_size, current_pose)
    else:
        rospy.logerr("Invalid path shape selected")
        return

    for _ in range(2):
        rospy.loginfo("Publishing path with shape: %s", args.shape)
        path_pub.publish(path)
        rospy.sleep(1)

if __name__ == "__main__":
    main()

