#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point32.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <costmap_converter/ObstacleArrayMsg.h>


int main(int argc, char ** argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "obstacle_publisher");
  ros::NodeHandle nh;
  ros::Rate r(10);

  ros::Publisher obs_pub = nh.advertise<costmap_converter::ObstacleArrayMsg>("/move_base/TebLocalPlannerROS/obstacles", 1);
  costmap_converter::ObstacleArrayMsg obstacle_msg;
  obstacle_msg.obstacles.resize(1);

  while(ros::ok()){

  //Add point obstacle
  obstacle_msg.header.stamp = ros::Time::now();
  obstacle_msg.obstacles[0].id = 1;
  obstacle_msg.obstacles[0].header.stamp = ros::Time::now();
  obstacle_msg.obstacles[0].polygon.points.resize(1);
  obstacle_msg.obstacles[0].polygon.points[0].x = 2.3;
  obstacle_msg.obstacles[0].polygon.points[0].y = 3.3;
  obstacle_msg.obstacles[0].polygon.points[0].z = 0;
  // obstacle_msg.obstacles[0].radius = 0.5;
  obstacle_msg.header.frame_id = "map"; // CHANGE HERE: odom/map

  //obstacle_msg.obstacles[0].velocities.twist.linear.x = 0.0001;
  //obstacle_msg.obstacles[0].velocities.twist.linear.y = 0.0001;

  obs_pub.publish(obstacle_msg); // publish
  r.sleep();
  }
  return 0;
}

