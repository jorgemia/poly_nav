#include <create_new_layers/line_layer.h> 
#include <pluginlib/class_list_macros.h> 


PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::LineLayer, costmap_2d::Layer) 

using costmap_2d::LETHAL_OBSTACLE; 

namespace simple_layer_namespace{ 

LineLayer::LineLayer() { // Constructor
line_sub_1 = _nh.subscribe("line_marker_1", 100, &LineLayer::landmarkspose1Callback, this);
line_sub_2 = _nh.subscribe("line_marker_2", 100, &LineLayer::landmarkspose2Callback, this);
curr_pose_sub = _nh.subscribe("robot_pose", 100, &LineLayer::robotposeCallback, this);
curr_node_sub = _nh.subscribe("current_node", 100, &LineLayer::currnodeCallback, this);
service = _nh.advertiseService("row_transition_mode", &LineLayer::change_row, this);
client = _nh.serviceClient<thorvald_2d_nav::sub_goal>("/row_transition_end_1");
} 

LineLayer::~LineLayer()
{
  if(dsrv_)
    delete dsrv_;
}

// Line points 
void LineLayer::landmarkspose1Callback (const visualization_msgs::Marker::ConstPtr& marker_msg1)
{
 if(marker_msg1->points.size() > 0){  
 l1[0].x = marker_msg1->points[0].x;
 l1[0].y = marker_msg1->points[0].y;
 l1[1].x = marker_msg1->points[1].x;
 l1[1].y = marker_msg1->points[1].y;
 }
}

// Line points 
void LineLayer::landmarkspose2Callback (const visualization_msgs::Marker::ConstPtr& marker_msg2)
{
 if(marker_msg2->points.size() > 0){  
 l2[0].x = marker_msg2->points[0].x;
 l2[0].y = marker_msg2->points[0].y;
 l2[1].x = marker_msg2->points[1].x;
 l2[1].y = marker_msg2->points[1].y;
 }
}

// Line points 
void LineLayer::robotposeCallback (const geometry_msgs::Pose::ConstPtr& pose_msg)
{
 robot_pose.position = pose_msg->position;
 robot_pose.orientation = pose_msg->orientation;
}

// Line points 
void LineLayer::currnodeCallback (const std_msgs::String::ConstPtr& curr_node_msg)
{
str_name = curr_node_msg->data;

if(str_name=="WayPoint4"){ // for WayPoint 3
// if(std::fabs(tf::getYaw(robot_pose.orientation))>2.7){ 
    end_row = false;
    end_row_transit.request.counter = 1;
    if (client.call(end_row_transit)) ROS_INFO("End of the row transition");
//  }
}

if(str_name=="WayPoint7"){ // for WayPoint 5
// if(std::fabs(tf::getYaw(robot_pose.orientation))>0.0){ 
    end_row = false;
    end_row_transit.request.counter = 1;
    if (client.call(end_row_transit)) ROS_INFO("End of the row transition");
//  }
}

}


// Reached end of row
bool LineLayer::change_row(thorvald_2d_nav::sub_goal::Request &req, thorvald_2d_nav::sub_goal::Response &res){
     ROS_INFO("transition service on time"); 
     end_row = true;
     return true;
}

void LineLayer::onInitialize() { 
ros::NodeHandle _nh("~/" + name_);
current_ = true; 
dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(_nh); 

dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind( &LineLayer::reconfigureCB, this, _1, _2); 
dsrv_->setCallback(cb); 
} 

void LineLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level){ 
if(end_row==false) enabled_ = config.enabled; 
if(end_row==true) enabled_ = false; 
}

void LineLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y) { 
if (!enabled_) return; 

// My addings
for (int i = 1; i <= Total_Points; i++) {
mark_x_[i] = (l1[0].x * (1 - (float(i) / Total_Points))) + (l1[1].x * (float(i) / Total_Points));
mark_y_[i] = (l1[0].y * (1 - (float(i) / Total_Points))) + (l1[1].y * (float(i) / Total_Points));
mark_x_1[i] = (l2[0].x * (1 - (float(i) / Total_Points))) + (l2[1].x * (float(i) / Total_Points));
mark_y_1[i] = (l2[0].y * (1 - (float(i) / Total_Points))) + (l2[1].y * (float(i) / Total_Points));

*min_x = std::min(*min_x, mark_x_[i]); 
*min_y = std::min(*min_y, mark_y_[i]); 
*max_x = std::max(*max_x, mark_x_[i]); 
*max_y = std::max(*max_y, mark_y_[i]);
}

} 

void LineLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) { 

 if (!enabled_) return; 
 unsigned int mx; 
 unsigned int my; 

 for (int i = 1; i <= Total_Points; i++) { // Update the costs of each grid cell (0.5 resolution)
  if(master_grid.worldToMap(mark_x_[i], mark_y_[i], mx, my)){ 
  master_grid.setCost(mx, my, LETHAL_OBSTACLE); } 
  if(master_grid.worldToMap(mark_x_1[i], mark_y_1[i], mx, my)){ 
  master_grid.setCost(mx, my, LETHAL_OBSTACLE); } 
  if(master_grid.worldToMap(mark_x_[i], mark_y_[i]+0.05, mx, my)){ 
  master_grid.setCost(mx, my, LETHAL_OBSTACLE); } 
  if(master_grid.worldToMap(mark_x_1[i], mark_y_1[i]+0.05, mx, my)){ 
  master_grid.setCost(mx, my, LETHAL_OBSTACLE); } 
  if(master_grid.worldToMap(mark_x_[i], mark_y_[i]+0.1, mx, my)){ 
  master_grid.setCost(mx, my, LETHAL_OBSTACLE); } 
  if(master_grid.worldToMap(mark_x_1[i], mark_y_1[i]+0.1, mx, my)){ 
  master_grid.setCost(mx, my, LETHAL_OBSTACLE); } 
  if(master_grid.worldToMap(mark_x_[i], mark_y_[i]-0.05, mx, my)){ 
  master_grid.setCost(mx, my, LETHAL_OBSTACLE); } 
  if(master_grid.worldToMap(mark_x_1[i], mark_y_1[i]-0.05, mx, my)){ 
  master_grid.setCost(mx, my, LETHAL_OBSTACLE); } 
  if(master_grid.worldToMap(mark_x_[i], mark_y_[i]-0.1, mx, my)){ 
  master_grid.setCost(mx, my, LETHAL_OBSTACLE); } 
  if(master_grid.worldToMap(mark_x_1[i], mark_y_1[i]-0.1, mx, my)){ 
  master_grid.setCost(mx, my, LETHAL_OBSTACLE); } 
}

} 

} // end namespace


