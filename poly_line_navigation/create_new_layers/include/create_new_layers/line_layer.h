#ifndef LINE_LAYER_H_
#define LINE_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>  // To get the line end points
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <thorvald_2d_nav/sub_goal.h>
std::string str_name;
 
namespace simple_layer_namespace
 {
  
  class LineLayer : public costmap_2d::Layer
  {
  public:
    LineLayer();
    virtual ~LineLayer(); 
    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
    virtual void landmarkspose1Callback (const visualization_msgs::Marker::ConstPtr& marker_msg1);
    virtual void landmarkspose2Callback (const visualization_msgs::Marker::ConstPtr& marker_msg2);
    virtual void robotposeCallback (const geometry_msgs::Pose::ConstPtr& pose_msg);
    virtual void currnodeCallback (const std_msgs::String::ConstPtr& curr_node_msg);
    virtual bool change_row(thorvald_2d_nav::sub_goal::Request &req, thorvald_2d_nav::sub_goal::Response &res);
    
    private:
     void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  
     // My addings
     ros::NodeHandle _nh;
     ros::Subscriber line_sub_1, line_sub_2, curr_pose_sub, curr_node_sub; 
     ros::ServiceServer service;
     ros::ServiceClient client;
     int Total_Points = 100;
     bool end_row = false;
     geometry_msgs::Point l1[2], l2[2];
     geometry_msgs::Pose robot_pose;
     double x1, y1, x2, y2, y3, y4;
     double euclidean_error_1, euclidean_error_2;
     double mark_x_[100], mark_y_[100];
     double mark_x_1[100], mark_y_1[100];
     thorvald_2d_nav::sub_goal end_row_transit;
     dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
   };
 }
 #endif
   
