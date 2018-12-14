#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <iostream>
#include <tf/tf.h>
#include <algorithm>
#include <cmath>
#include <math.h>       /* fabs */
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>

// ROS message includes
#include <thorvald_2d_nav/scan_detected_line.h>
#include <thorvald_2d_nav/sub_goal.h>
#include <thorvald_2d_nav/landmarks.h>

#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// RANSAC Parameters 
geometry_msgs::Point n;
int k = 1000; // iterations need to find the best model
double thershold = 0.05;
int d = 35; // nearby point to fit the line

int max_laser_range = 5.0;
double best_model;
size_t num_ranges;
double x_1[1080], y_1[1080], angle_1[1080], x_2[1080], y_2[1080], angle_2[1080];
sensor_msgs::LaserScan scan_msg_main;
double current_range_1, current_range_2;
int count_i_1[1080], count_i_2[1080];
visualization_msgs::Marker line_strip_1, line_strip_2, final_line;
thorvald_2d_nav::scan_detected_line meas_pts;
thorvald_2d_nav::landmarks landmarks_pos, prev_landmarks_pose;
tf2_ros::Buffer tfBuffer, tfBuffer1; 
geometry_msgs::Pose thorvald_pose, line_pose, hokuyo_pose;
thorvald_2d_nav::sub_goal end_row_check, end_row_check_1, end_row_check_2;
int first_l = 0, second_l = 0, first_r = 0, second_r = 0, stop_1 = 0, stop_2 = 0;
int row_transit_mode = 0;

// Our "data".
struct Point {
  double real_x; double real_y;
};

// Our "model".
struct Line {
  double m; double b;
};

Point best_inlierpoints_1[1080];
Point current_best_inlierpoints_1[1080];
Line final_line_1;
Point current_Index_1[2];

Point best_inlierpoints_2[1080];
Point current_best_inlierpoints_2[1080];
Line final_line_2;
Point current_Index_2[2];

//Transform varibles
geometry_msgs::Point line_path[2];
geometry_msgs::TransformStamped transformStamped, transformStamped1;
geometry_msgs::PoseStamped curr_pose, curr_pose_trans, left_line_[2], left_line_trans[2], right_line_[2], right_line_trans[2];
geometry_msgs::Pose left_l_c, right_l_c;

// dummy variables
int p_1 = 1, l_1 = 1, c_1 = 0, a_1 = 0, b_1 = 0, p_2 = 1, l_2 = 1, c_2 = 0, a_2 = 0, b_2 = 0;
int final_count_1 = 0, final_count_2 = 0, final_count_3 = 0, final_count_4 = 0, final_count_5 = 0, final_count_6 = 0; 
int finale = 0;
double row_end = 0, yaw = 0, yaw1 = 0, row_no = 1;
int end_row_reach = 0, line_pt = 0, lpose_tmp = 0, pose_tmp = 0, line_tolerance = 2.0;
double min_range_view = 4.0, max_range = 30.0;
int end_line = 0, end_row = 0, finale_1 = 0, finale_2 = 0, land_check = 0;
bool line_found_1 = false, line_found_2 = false, new_row = false;
visualization_msgs::Marker empty_line_strip_1, empty_line_strip_2, empty_final_line;
thorvald_2d_nav::scan_detected_line empty_meas_pts; 
thorvald_2d_nav::landmarks empty_landmarks_pos;
double slope_1 = 0, slope_2 = 0, slope_3 = 0, slope_4 = 0, line_local[3];
int aIndex_1 = 0, aIndex_2 = 0, bIndex_1 = 0, bIndex_2 = 0, init_pose = 0;

// Controller Variables

//Callback Initialzations
geometry_msgs::PoseStamped thor_est, thor_est_trans;
thorvald_2d_nav::landmarks landmarks_pose;
geometry_msgs::TransformStamped transformStamped2;
tf2_ros::Buffer tfBuffer2;

//Rosservice parameters
thorvald_2d_nav::sub_goal goal_count; 
// int row_no = 0;
bool last_set_check = false, next_row_check = true;

//Controller Parameters
geometry_msgs::Point mini_goal_pts;
int Total_Points = 10;
geometry_msgs::Pose Points[20];
geometry_msgs::Twist est_twist;
double position_error, q_x , q_y, lastError = 0, goal_yaw = 0;
std_msgs::Float64 angular_error;
double K_d = 0.05, K_p = 0.1, K_i = 20.0;
bool mini_goal = false;
double dist_d[2], dist_pt_1 = 0, dist_pt_2 = 0, dist_pt_f = 0, dist_pt_3 = 0, dist_pt_4 = 0, dist_pt_f1 = 0;
double  omega = 0, omega_exp = 0;
double asq = 0, bsq = 0, csq = 0, ang_err = 0;

// dummy variables
int counter_line = 0, counter_pose = 0, counter_1 = 0, line_count = 0, row_count = 0, c = 1;
double linear_velocity = 0.2, angular_velocity = 0;
double dt = 0, intregral = 0, tolerance = 1.5;

// Functions
Point Line_detection_1(geometry_msgs::Pose thor_pose);
Point Line_detection_2(geometry_msgs::Pose thor_pose);
double control_law(geometry_msgs::Pose thor_est);
ros::Publisher marker_pub_1, marker_pub_2, marker_pub_3, point_pub, twist_gazebo, landmarks_pub, a_err;
