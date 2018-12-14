#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <algorithm>
#include <std_msgs/String.h>
#include <iostream>
#include <tf/tf.h>
#include <algorithm>    // std::max
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>
// ROS message includes
#include <thorvald_2d_nav/scan_detected_line.h>
#include <thorvald_2d_nav/landmarks.h>
#include <thorvald_2d_nav/sub_goal.h>

#define INF 1000
using namespace Eigen;

// Initialization for callbacks
geometry_msgs::Pose thor_pose;
geometry_msgs::Twist odom_vel;
nav_msgs::Odometry robot_pose;
thorvald_2d_nav::landmarks landmarks_pose, prev_landmarks_pose;
thorvald_2d_nav::sub_goal goal_count; 
geometry_msgs::Twist twist_msg;
thorvald_2d_nav::scan_detected_line actual_meas, prev_meas_pts;
int total_points = 4, total_landmarks = 20; 
double d = 0.5; // distance between two wheels
double lambda_x = 0, lambda_y = 0, q = 0, dt = 0, bearing_norm = 0, range_1 = 0;
bool landmarks_observed = false;
ros::Time current_time, last_time;
int init_pose = 1, end_row_info = 0, finale = 0;
int f_n = -1, curr_f_n = 0, row_transit_mode = 0, th_dist = 0.5;
bool line_SLAM_status = true;

// Markers
nav_msgs::Odometry thorvald_estimated_pose;
visualization_msgs::Marker landmark_strip_1, landmark_strip_2;

// Initialization for localization
double yaw, motion_noise = 0.000000001, measurement_noise = 0.00000001, sub_goal_thershold = 0.05;
MatrixXd mu = MatrixXd::Zero(2*total_landmarks+3,1); // mu
MatrixXd line_local(total_landmarks,2); // line_local
MatrixXd line_local_fixed(total_landmarks,2); // line_local_fixed
MatrixXd Z = MatrixXd::Zero(2*total_points,1); // Z
MatrixXd expectedZ = MatrixXd::Zero(2*total_points,1); // expectedZ
MatrixXd K = MatrixXd::Zero((2*total_landmarks+3),2*total_points); // Kalman gain
MatrixXd diff = MatrixXd::Zero(2*total_points,1); // error
MatrixXd H = MatrixXd::Zero(2*total_points,(2*total_landmarks+3)); // H - Jacobian
MatrixXd line_pho = MatrixXd::Zero(total_points,1); // polar co-ordinates
MatrixXd line_theta = MatrixXd::Zero(total_points,1); // polar co-ordinates
MatrixXd robSigma    = MatrixXd::Zero(3,3);
MatrixXd robMapSigma = MatrixXd::Zero(3,(2*total_landmarks));
MatrixXd mapSigma    = 0.01 * MatrixXd::Identity((2*total_landmarks), (2*total_landmarks));
MatrixXd cov = MatrixXd::Zero((2*total_landmarks+3),(2*total_landmarks+3));
MatrixXd R = MatrixXd::Zero((2*total_landmarks+3),(2*total_landmarks+3))* 1e-15; // Motion Noise
MatrixXd Q = MatrixXd::Identity((2*total_points),(2*total_points))* measurement_noise; // Measurement Noise
MatrixXd ma_dist= MatrixXd::Zero(1,1);

//initialize velocity variables
double vx = 0.0, vy = 0.0, vth = 0.0;
