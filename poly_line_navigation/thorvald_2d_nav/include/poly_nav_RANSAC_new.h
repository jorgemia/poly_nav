#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <iostream>
#include <tf/tf.h>
#include <algorithm>
#include <cmath>
#include <math.h>       /* fabs */
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> /*doTransform*/

// ROS message includes
#include <thorvald_2d_nav/scan_detected_line.h>
#include <thorvald_2d_nav/sub_goal.h>
#include <thorvald_2d_nav/landmarks.h>


class Ransac{

public:
// RANSAC Parameters 
int k = 100; // iterations need to find the best model
double thershold = 0.05;
int d = 10; // nearby point to fit the line
size_t num_ranges;
int max_laser_range = 6.5;
sensor_msgs::LaserScan scan_msg_main;
struct Line { double m; double b; }; // Our "model".
struct Point { double real_x; double real_y;}; // Our "data".
Point best_inlier_set[720];
Point current_best_inlier_set[720];
Line model;

// initialization
geometry_msgs::Pose thorvald_pose, line_pose, hokuyo_pose;
geometry_msgs::Point line_[4], line_trans_[4], prev_line_trans_[4];
geometry_msgs::TransformStamped transformStamped;
thorvald_2d_nav::landmarks landmarks_pose, prev_landmarks_pose, empty_landmarks_pose;
thorvald_2d_nav::scan_detected_line meas_pts, prev_meas_pts, empty_meas_pts;
visualization_msgs::Marker line_strip[3], empty_line_strip[3];
double x[720] = {0}, y[720] = {0}, theta[720] = {0}, scan_pts_index_array[720] = {0}, yaw = 0, error = 0, d_line = 0, d_ther = 0.2, w_p = 0.1, w_b = 0.1;
int s_pt = 0, line_1_pt_1 = 0, line_1_pt_2 = 0, aIndex_1 = 0, aIndex_2 = 0, bIndex_1 = 0, bIndex_2 = 0;
int n = 0, index_no = 0, total_pts = 0, best_no_inliers = 0, first_pt = 0, second_pt = 0, row_transit_mode = 0, one_time = 0;
bool one_line_found = false, both_lines_found = false, row_follow_mode = true;

//Controller Parameters
geometry_msgs::Point mini_goal_pts;
geometry_msgs::Twist est_twist;
geometry_msgs::Pose Points[20];
int Total_Points = 10;
double position_error = 0, q_x = 0 , q_y = 0, lastError = 0, goal_yaw = 0, line_count = 0, angular_velocity = 0, row_count = 0, tolerance = 1.2;
double K_d = 0.05, K_p_1 = 0.05, K_p_2 = 0.05, K_i = 20.0;
bool mini_goal = false;
std_msgs::Float64 angular_error;
thorvald_2d_nav::sub_goal end_row_check, end_row_check_1;

ros::NodeHandle nh_;
ros::Publisher marker_pub_1, marker_pub_2, marker_pub_3, point_pub, twist_gazebo, landmarks_pub, a_err;
ros::Subscriber scan_sub, pose_sub, hpose_sub, lpose_sub;
ros::ServiceServer service_obj;
ros::ServiceClient client, client1;

// Functions
void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
void poseCallback(const geometry_msgs::Pose::ConstPtr& pose_msg);
void hposeCallback(const geometry_msgs::Pose::ConstPtr& hpose_msg);
void lposeCallback(const nav_msgs::Odometry::ConstPtr& lpose_msg);
bool row_transition(thorvald_2d_nav::sub_goal::Request &req, thorvald_2d_nav::sub_goal::Response &res);

Ransac();

double normalizeangle(double bearing);
void initialize();
void move();
void create_markers(int id_number);
void calculate_measurements();
void controller();
};
