#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <algorithm>
#include <std_msgs/String.h>
#include <iostream>
#include <tf/tf.h>
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>

// ROS message includes
#include <thorvald_2d_nav/sub_goal.h>
#include <thorvald_2d_nav/landmarks.h>

#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>

//Callback Initialzations
geometry_msgs::PoseStamped thor_est, thor_est_trans;
thorvald_2d_nav::landmarks landmarks_pose;
geometry_msgs::TransformStamped transformStamped;
tf2_ros::Buffer tfBuffer;

//Rosservice parameters
thorvald_2d_nav::sub_goal goal_count; 
int row_no = 0;
bool next_row_check = true;

//Controller Parameters
geometry_msgs::Point mini_goal_pts;
int Total_Points = 10;
geometry_msgs::Pose Points[20];
geometry_msgs::Twist est_twist;
double yaw, position_error, angular_error, q_x , q_y, lastError = 0;
double K_d = 2000000000000000.0, K_p = 10.0, K_i = 20.0;
bool mini_goal = false;
double dist_d[2], dist_pt_1, dist_pt_2, dist_pt_f, dist_pt_3, dist_pt_4, dist_pt_f1;
double slope_1, slope_2, omega, omega_exp;
double asq, bsq, csq, ang_err;

// dummy variables
int counter_line = 0, counter_pose = 0, counter_1 = 0, line_count = 0, row_count = 0, c = 1;

