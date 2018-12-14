#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <algorithm>
#include <std_msgs/String.h>
#include <iostream>
#include <tf/tf.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

// ROS message includes
#include <thorvald_2d_nav/sub_goal.h>

#include "tf2_ros/message_filter.h"
#include "tf/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//Callback parameters
geometry_msgs::Twist est_twist;
double yaw = 0, dt = 0;
geometry_msgs::PoseStamped thor_est, thor_est_trans;
geometry_msgs::Pose hokuyo_pose;

//Transform parameters
geometry_msgs::Pose goal_pt[2];
std_msgs::Float64 ang_error;
tf2_ros::Buffer tfBuffer, tfBuffer1;
geometry_msgs::TransformStamped transformStamped, transformStamped1;
thorvald_2d_nav::sub_goal end_row_transit, end_row_transit_1;
geometry_msgs::PoseStamped pole_[3], pole_trans_[3];

//Parameters for pole detection
double min_range_left = 2.0, min_range_right = 2.5;
sensor_msgs::LaserScan scan_msg_main, scan_msg;
size_t num_ranges;
bool goal_found = false;
double sum_x_1 = 0, sum_y_1 = 0;
geometry_msgs::Pose nearest_pole, next_nearest_pole, farthest_pole;
double min_itr_1, max_itr_1, min_itr_2, max_itr_2;
int turn_side = 1; // 1 for RIGHT, 2 for LEFT
enum TURN {RIGHT = 1, LEFT = 2};

//Pole Markers
visualization_msgs::Marker marker_[5];
visualization_msgs::Marker empty_marker_[5];

//dummy variables
double linear_velocity = 0.4, angular_velocity = 0.0;
double goal_range, goal_bearing, a = 0;
bool pole_detect = false, turn_90 = false, turning_180 = false, in_place_turn = false; 
int goal_transit = 1, goal_transit1 = 1, c = 0, row_transit = 1, row_transit_mode = 0, count_1 = 0;
double min_goal_range = 0.1, max_goal_range = 10.0;
