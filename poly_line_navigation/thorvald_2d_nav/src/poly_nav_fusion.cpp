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
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include <random>
#include <string>
#include <assert.h>

using namespace Eigen;
int n1 = 1, total_landmarks = 2;

// Thorvald pose structure
struct thorvald_pose_struct{
bool landmarks_observed;
MatrixXd mu = MatrixXd::Zero(3,1); // mu
MatrixXd sigma = MatrixXd::Zero(3,3); // sigma
};

thorvald_pose_struct thor_pose, thor_pose_est;
geometry_msgs::Pose true_pose, current_waypoint_;
geometry_msgs::PoseArray waypts_pos;
nav_msgs::Odometry thor_est_g, fus_pos_p, thor_est_l;
MatrixXd expp = MatrixXd::Zero(1,1);
MatrixXd weight_1 = MatrixXd::Zero(1,1);
MatrixXd weight_2 = MatrixXd::Zero(1,1);
MatrixXd zi =  MatrixXd::Zero(3,1);
MatrixXd weight = MatrixXd::Zero(1,1);
MatrixXd p_j_0 = MatrixXd::Zero(1,1);
MatrixXd p_j_1 = MatrixXd::Zero(1,1);
int connected_sensor = 1;
enum sensor_type {GPS = 1, LASER = 2};
double yaw, yaw_l ,yaw_g;
int sub_check_1 = 0, sub_check_2 = 0, sub_check_3 = 0;

// update step variables
double motion_noise = 0.001, measurement_noise = 0.001;
MatrixXd H = MatrixXd::Zero(3,3); // H - Jacobian
MatrixXd K = MatrixXd::Zero(3,3); // Kalman gain
MatrixXd Z = MatrixXd::Zero(3,1); // Z
MatrixXd expectedZ = MatrixXd::Zero(3,1); // expectedZ
MatrixXd diff = MatrixXd::Zero(3,1); // error
MatrixXd Q_t = MatrixXd::Identity(3,3)* measurement_noise; // Measurement Noise 
MatrixXd Q = MatrixXd::Zero(3,3)* measurement_noise; // Measurement Process 
MatrixXd C = MatrixXd::Zero(3,3); // C for weight

void thorposeCallback(const geometry_msgs::Pose::ConstPtr& curr_pose){
true_pose.position = curr_pose->position;
true_pose.orientation = curr_pose->orientation;

tf::Quaternion quat(true_pose.orientation.x,true_pose.orientation.y, true_pose.orientation.z, true_pose.orientation.w);
quat = quat.normalize();
yaw = tf::getYaw(quat);
sub_check_1 = 1;
}

void estposeCallback(const nav_msgs::Odometry::ConstPtr& est_pose){
thor_est_l.pose.pose.position = est_pose->pose.pose.position;
thor_est_l.pose.pose.orientation = est_pose->pose.pose.orientation;

tf::Quaternion quat_l(thor_est_l.pose.pose.orientation.x,thor_est_l.pose.pose.orientation.y, thor_est_l.pose.pose.orientation.z, thor_est_l.pose.pose.orientation.w);
quat_l = quat_l.normalize();
yaw_l = tf::getYaw(quat_l);
sub_check_2 = 1;
}

/*void estposeCallback1(const nav_msgs::Odometry::ConstPtr& est_pose_g){
thor_est_g.pose.pose = est_pose_g->pose.pose;

   ROS_INFO("here2");
tf::Quaternion quat_g(thor_est_g.pose.pose.orientation.x,thor_est_g.pose.pose.orientation.y, thor_est_g.pose.pose.orientation.z, thor_est_g.pose.pose.orientation.w);
quat_g = quat_g.normalize();
yaw_g = tf::getYaw(quat_g);
sub_check_3 = 1;
} */

void wayptCallback(const geometry_msgs::PoseArray::ConstPtr& waypt_sub_pose){
waypts_pos.poses.resize(waypt_sub_pose->poses.size());

std::cout << waypt_sub_pose->poses.size() << std::endl;
if(waypt_sub_pose->poses.size()>0){
 for(int wd = 0; wd < waypt_sub_pose->poses.size(); wd++){
 waypts_pos.poses[wd] = waypt_sub_pose->poses[wd];
 }
}

}


double normalizeangle(double bearing){
    if (bearing < -M_PI) {
        bearing += 2*M_PI;
    } else if (bearing > M_PI) {
        bearing -= 2*M_PI;
    }
}

MatrixXd measurement_model(){

switch(connected_sensor){
case GPS:// For GPS
H(0,0) = 1; 
H(1,1) = 1; 
H(2,2) = 0; 
break;

case LASER: // For Laser
H(0,0) = 1; 
H(1,1) = 1; 
H(2,2) = 1; 
break;
}

return H;
}

struct thorvald_pose_struct correction_step_gps(nav_msgs::Odometry thor_pose_g, geometry_msgs::Pose current_waypt){

struct thorvald_pose_struct thor_pos_g;

  connected_sensor = 1;

 // calculate jacobian w.r.t landmark pose
  H = measurement_model();
 
 // Kalman Gain
  K = thor_pos_g.sigma*H.transpose()*(H*thor_pos_g.sigma*H.transpose()+Q).inverse();

 // Innovation
  Z(0,0) = thor_pose_g.pose.pose.position.x;
  Z(1,0) = thor_pose_g.pose.pose.position.y;
 
  expectedZ(0,0) = current_waypt.position.x;
  expectedZ(1,0) = current_waypt.position.y;
  diff = expectedZ - Z;
  
 // Finish the correction step by computing the new mu and sigma.
  thor_pos_g.mu = thor_pos_g.mu + K*diff;
  thor_pos_g.sigma =  (MatrixXd::Identity(3,3)- K*H)*thor_pos_g.sigma;
  thor_pos_g.mu(2,0)= normalizeangle(thor_pos_g.mu(2,0));

return thor_pos_g;
}

struct thorvald_pose_struct correction_step_laser(nav_msgs::Odometry thor_pose_l_1,geometry_msgs::Pose current_waypt){

struct thorvald_pose_struct thor_pos_l;

  connected_sensor = 2;

 // calculate jacobian w.r.t landmark pose
  H = measurement_model();

 // Kalman Gain
  K = thor_pos_l.sigma*H.transpose()*(H*thor_pos_l.sigma*H.transpose()+Q).inverse();
 
 // Innovation
  Z(0,0) = thor_pose_l_1.pose.pose.position.x;
  Z(1,0) = thor_pose_l_1.pose.pose.position.y;
  Z(2,0) = yaw;

  expectedZ(0,0) = current_waypt.position.x;
  expectedZ(1,0) = current_waypt.position.y;
  expectedZ(2,0) = atan2(current_waypt.position.y,current_waypt.position.x);
  diff = expectedZ - Z;
 
 // Finish the correction step by computing the new mu and sigma.
  thor_pos_l.mu = thor_pos_l.mu + K*diff;
  thor_pos_l.sigma =  (MatrixXd::Identity(3,3)- K*H)*thor_pos_l.sigma;
  thor_pos_l.mu(2,0)= normalizeangle(thor_pos_l.mu(2,0));

return thor_pos_l;
}


MatrixXd compute_weight(MatrixXd innovation){
C = H*thor_pose.sigma*H.transpose() + Q;
expp = (-0.5*innovation.transpose()*C.inverse()*innovation).exp();
weight = (1/ (pow((2*M_PI),n1/2) * sqrt(C.determinant()))) * expp;
return weight;
}

struct thorvald_pose_struct fusion_model(struct thorvald_pose_struct thor_pose_f, MatrixXd weigh_1, MatrixXd weigh_2){

// normalizing weight
p_j_0 = weigh_1*(weigh_1+weigh_2).inverse();
p_j_1 = weigh_2*(weigh_1+weigh_2).inverse();

double p_j_0_n = p_j_0(0.0);
double p_j_1_n = p_j_1(0,0);

thor_pose_est.landmarks_observed = thor_pose_f.landmarks_observed;	

// final state and co-variance model
thor_pose_est.mu = thor_pose_f.mu*p_j_0 + thor_pose_f.mu*p_j_1;
zi = thor_pose_est.mu - thor_pose_f.mu;
thor_pose_est.sigma = p_j_0_n*(thor_pose_f.sigma+(zi*zi.transpose())) + p_j_1_n*(thor_pose_f.sigma+(zi*zi.transpose()));
return thor_pose_est;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "poly_nav_fusion");
  ros::NodeHandle n;
  ros::Rate r(10.0);

  // Subscribers
  ros::Subscriber pose_sub = n.subscribe("/thorvald_pose", 100, thorposeCallback);
  ros::Subscriber pose_sub1 = n.subscribe("/line_pose", 100, estposeCallback);
 // ros::Subscriber pose_sub2 = n.subscribe("/odometry/gps", 100, estposeCallback1);
  ros::Subscriber waypt_sub = n.subscribe("/way_pts", 100, wayptCallback);

  // Publishers
  ros::Publisher fus_pose_pub = n.advertise<nav_msgs::Odometry>("/fusion_pose", 100);
  struct thorvald_pose_struct thor_g, thor_l; 
  std::string filename;
  n.param("filename", filename, filename);

  while (ros::ok()){

  // getwaypoints(filename);

  ros::spinOnce();

  if((sub_check_1>0)&&(waypts_pos.poses.size()>0)){

  // current waypoint check
   for(int i=1;i<waypts_pos.poses.size();i++){
     if((true_pose.position.x-waypts_pos.poses[i].position.x)<0.1) current_waypoint_ = waypts_pos.poses[i];
   }
  // update step
 // thor_g = correction_step_gps(thor_est_g, current_waypoint_);
  thor_l = correction_step_laser(thor_est_l , current_waypoint_);

  // compute weight
   weight_1 = compute_weight(diff);
   weight_2(0,0) = 0;

 //  weight_2 = compute_weight(thor_pose, diff);

  // fusion model
  thor_pose = fusion_model(thor_pose, weight_1, weight_2);
 // ROS_INFO("over");

  fus_pos_p.pose.pose.position.x = thor_pose.mu(0,0);
  fus_pos_p.pose.pose.position.y = thor_pose.mu(1,0);
  fus_pos_p.pose.pose.orientation = tf::createQuaternionMsgFromYaw(thor_pose.mu(2,0));

  fus_pose_pub.publish(fus_pos_p);
  }
  r.sleep();
  }
  return 0;
}

