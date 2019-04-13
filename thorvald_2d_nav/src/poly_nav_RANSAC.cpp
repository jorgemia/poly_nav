#include "poly_nav_RANSAC.h"
double K_p_1 = 0.05, K_p_2 = 0.05;

// Laser Scan data
void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
  num_ranges = scan_msg->ranges.size();
  scan_msg_main.header.stamp = scan_msg->header.stamp;
  scan_msg_main.ranges = scan_msg->ranges;
  scan_msg_main.angle_min = scan_msg->angle_min;
  scan_msg_main.angle_increment = scan_msg->angle_increment;
 }  // callback end

// thorvald pose data
void poseCallback (const geometry_msgs::Pose::ConstPtr& pose_msg)
{

if((!std::isnan(pose_msg->position.x)) || (!std::isnan(pose_msg->position.y))){
thorvald_pose.position = pose_msg->position;
thorvald_pose.orientation = pose_msg->orientation;
tf::Quaternion quat(thorvald_pose.orientation.x,thorvald_pose.orientation.y, thorvald_pose.orientation.z, thorvald_pose.orientation.w);
quat = quat.normalize();
yaw = tf::getYaw(quat);

pose_tmp = 1;
}

} // callback end

// thorvald pose data
void hposeCallback (const geometry_msgs::Pose::ConstPtr& hpose_msg)
{

hokuyo_pose.position = hpose_msg->position;
hokuyo_pose.orientation = hpose_msg->orientation;

pose_tmp = 1;
} // callback end


// thorvald pose data
void lposeCallback (const nav_msgs::Odometry::ConstPtr& lpose_msg)
{
line_pose.position = lpose_msg->pose.pose.position;
line_pose.orientation = lpose_msg->pose.pose.orientation;

tf::Quaternion quat1(line_pose.orientation.x,line_pose.orientation.y, line_pose.orientation.z, line_pose.orientation.w);
quat1 = quat1.normalize();
yaw1 = tf::getYaw(quat1);

lpose_tmp = 1;
} // callback end

double normalizeangle(double bearing) {
  if (bearing < -M_PI) {
    bearing += 2 * M_PI;
  } else if (bearing > M_PI) {
    bearing -= 2 * M_PI;
  }
}

// RANSAC for line detection
Point Line_detection_1(geometry_msgs::Pose thor_pose) {
  
 for (int ki = 0; ki <= k; ki++) {  // Number of iterations

   // x, y,theta calculation
  for (int i_1 = 0; i_1 <= (num_ranges/2); i_1++) {
    angle_1[i_1] = scan_msg_main.angle_min + i_1 * scan_msg_main.angle_increment;
    x_1[i_1] = scan_msg_main.ranges[i_1] * cos(angle_1[i_1]);
    y_1[i_1] = scan_msg_main.ranges[i_1] * sin(angle_1[i_1]); 

   if (!std::isnan(x_1[i_1]) && !std::isnan(y_1[i_1]) && (scan_msg_main.ranges[i_1] < max_laser_range) && (std::fabs(y_1[i_1]) < 1.5)) {
      count_i_1[l_1] = i_1;
      l_1 = l_1 + 1;
    }  // storing the ith value with pre-conditions

  }  // storing the range values (end for loop)

  if(l_1 > d){ // bounding box check
  l_1 = l_1 - 1;
  int i_corr_x_1 = rand() % (l_1);
  int i_corr_y_1 = rand() % (l_1);
  aIndex_1 = count_i_1[i_corr_x_1];
  bIndex_1 = count_i_1[i_corr_y_1];

  // sending the scan data
  Line model_1;
  int count_1 = 0;

  if ((aIndex_1 != bIndex_1) && (y_1[aIndex_1] != 0) && (x_1[aIndex_1] != 0) &&
      (y_1[bIndex_1] != 0) && (x_1[bIndex_1] != 0)) {
    model_1.m = (y_1[bIndex_1] - y_1[aIndex_1]) / (x_1[bIndex_1] - x_1[aIndex_1]);
    model_1.b = y_1[aIndex_1] - model_1.m * x_1[aIndex_1];
  }

  else {
    count_1 = 1;
  }

  if (count_1 == 0) {  // check for aIndex and bIndex values

    for (int i_1 = 0; i_1 < (l_1); i_1++) {
      int index_1 = count_i_1[i_1];
      double error_1 = y_1[index_1] - (model_1.m * x_1[index_1] + model_1.b);  // error calculation

      if ((thershold > error_1) && (-thershold < error_1) && (error_1 != 0)) {  // check for threshold
        current_best_inlierpoints_1[p_1].real_x = x_1[index_1];
        current_best_inlierpoints_1[p_1].real_y = y_1[index_1];
        p_1 = p_1 + 1;
      }
    }  // for loop end

    final_count_1 = p_1 - 1;
    l_1 = 0;

    if (final_count_1 > d) {  // selecting the inliers with max of points
      if ((final_count_1 > final_count_2) && (std::fabs(y_1[aIndex_1]) < 1.5) && (std::fabs(y_1[bIndex_1]) < 1.5)) {
        if (scan_msg_main.ranges[aIndex_1] < scan_msg_main.ranges[bIndex_1]) {
          left_line_[0].pose.position.x = x_1[aIndex_1];
          left_line_[0].pose.position.y = y_1[aIndex_1];
          left_line_[1].pose.position.x = x_1[bIndex_1];
          left_line_[1].pose.position.y = y_1[bIndex_1];
          first_l = aIndex_1;
          second_l = bIndex_1;
        } else {
          left_line_[0].pose.position.x = x_1[bIndex_1];
          left_line_[0].pose.position.y = y_1[bIndex_1];
          left_line_[1].pose.position.x = x_1[aIndex_1];
          left_line_[1].pose.position.y = y_1[aIndex_1];
          first_l = bIndex_1;
          second_l = aIndex_1;
        }

        final_count_2 = final_count_1;
        line_found_1 = true;
      }
    }  // max inliers selection end

    // Assigning variables to zero to start over
    p_1 = 0;
    final_count_2 = 0;
   }  // check for aIndex and bIndex values end

   } // bounding box check
  } //number of iterations

}

Point Line_detection_2(geometry_msgs::Pose thor_pose) {

 for (int ki = 0; ki <= k; ki++) {  // Number of iterations

  // x, y, theta calculation
  for (int i_2 = (num_ranges / 2); i_2 <= num_ranges; i_2++) {
    angle_2[i_2] = scan_msg_main.angle_min + i_2 * scan_msg_main.angle_increment;

    x_2[i_2] = scan_msg_main.ranges[i_2] * cos(angle_2[i_2]);
    y_2[i_2] = scan_msg_main.ranges[i_2] * sin(angle_2[i_2]); 

    if (!std::isnan(x_2[i_2]) && !std::isnan(y_2[i_2]) && (scan_msg_main.ranges[i_2] < max_laser_range) && (std::fabs(y_2[i_2]) < 1.5)) {
      count_i_2[l_2] = i_2;
      l_2 = l_2 + 1;
    }  // storing the ith value with pre-conditions
  }    // storing the range values (end for loop)

  if(l_2 > d){ // bounding box check

  l_2 = l_2 - 1;
  int i_corr_x_2 = rand() % (l_2);
  int i_corr_y_2 = rand() % (l_2);
  aIndex_2 = count_i_2[i_corr_x_2];
  bIndex_2 = count_i_2[i_corr_y_2];

  // sending the scan data
  Line model_2;
  int count_2 = 0;

  if ((aIndex_2 != bIndex_2) && (y_2[aIndex_2] != 0) && (x_2[aIndex_2] != 0) && (y_2[bIndex_2] != 0) && (x_2[bIndex_2] != 0)) {
    model_2.m = (y_2[bIndex_2] - y_2[aIndex_2]) / (x_2[bIndex_2] - x_2[aIndex_2]);
    model_2.b = y_2[aIndex_2] - model_2.m * x_2[aIndex_2];
  }

  else {
    count_2 = 1;
  }

  if (count_2 == 0) {  // check for aIndex and bIndex values

    for (int i_2 = 0; i_2 < (l_2); i_2++) {
      int index_2 = count_i_2[i_2];
      double error_2 = y_2[index_2] - (model_2.m * x_2[index_2] + model_2.b);  // error calculation

      if((thershold > error_2) && (error_2 > (-thershold)) && (error_2 != 0)){
        current_best_inlierpoints_2[p_2].real_x = x_2[index_2];
        current_best_inlierpoints_2[p_2].real_y = y_2[index_2];
        p_2 = p_2 + 1;
      }
    }  // for loop end

    final_count_4 = p_2 - 1;
    l_2 = 0;

    if (final_count_4 > d) {  // selecting the inliers with max of points
      if (final_count_4 > final_count_5 && (std::fabs(y_2[aIndex_2]) < 1.2) && (std::fabs(y_2[bIndex_2]) < 1.2)) {
        
         if (scan_msg_main.ranges[aIndex_2] < scan_msg_main.ranges[bIndex_2]) {
          right_line_[0].pose.position.x = x_2[aIndex_2];
          right_line_[0].pose.position.y = y_2[aIndex_2];
          right_line_[1].pose.position.x = x_2[bIndex_2];
          right_line_[1].pose.position.y = y_2[bIndex_2];
          first_r = aIndex_2;
          second_r = bIndex_2;
        } else {
          right_line_[0].pose.position.x = x_2[bIndex_2];
          right_line_[0].pose.position.y = y_2[bIndex_2];
          right_line_[1].pose.position.x = x_2[aIndex_2];
          right_line_[1].pose.position.y = y_2[aIndex_2];
          first_r = bIndex_2;
          second_r = aIndex_2;
        }

        final_count_5 = final_count_4;
        line_found_2 = true;
      }
    }  // max inliers selection end

    // Assigning variables to zero to start over
    p_2 = 0;
    final_count_5 = 0;
   }  // check for aIndex and bIndex values end
  } // bounding box check
 } //number of iterations
}

double control_law(geometry_msgs::Pose curr_pose) {

  // calculation of error
   if(row_transit_mode==1){
   q_x = curr_pose.position.x - mini_goal_pts.x;
   q_y = curr_pose.position.y - mini_goal_pts.y;
   }
   else{
   q_x = mini_goal_pts.x - curr_pose.position.x;
   q_y = mini_goal_pts.y - curr_pose.position.y; 
   }

  position_error = sqrt(pow(q_x, 2) + pow(q_y, 2));
  angular_error.data = (atan2(q_y, q_x) - yaw);

/*  tf::Quaternion quat_r(curr_pose.orientation.x,curr_pose.orientation.y, curr_pose.orientation.z, curr_pose.orientation.w);
  quat_r = quat_r.normalize();
  angular_error.data = normalizeangle(goal_yaw - tf::getYaw(quat_r));
*/  

  a_err.publish(angular_error);

  if (q_y < 0) omega = K_p_2 * q_y + 0.02 * (q_y - lastError);
  else omega = K_p_1 * q_y + 0.02 * (q_y - lastError);
  std::cout << " " << q_y << " " << omega << std::endl; 
  std::cout << " " << K_p_1 << " " << K_p_2 << std::endl; 

  lastError = q_y;

  if (omega > M_PI) omega -= 2 * M_PI;
  if (omega < -M_PI) omega += 2 * M_PI;

  return omega;
}

bool row_transition(thorvald_2d_nav::sub_goal::Request &req, thorvald_2d_nav::sub_goal::Response &res)
   {
     end_row = end_row + req.counter; 
     landmarks_pos.row_number = landmarks_pos.row_number + req.counter;
     landmarks_pos.feature_no = 0;
     row_transit_mode = row_transit_mode + 1;
 
     if(row_transit_mode==2){
     prev_landmarks_pose.x[0] = 0.0;
     prev_landmarks_pose.x[2] = 0.0;
     K_p_1 = 0.15;
     K_p_2 = 0.075;
    }

     if(row_transit_mode==1){
     prev_landmarks_pose.x[0] = 25.0;
     prev_landmarks_pose.x[2] = 25.0;
     max_laser_range = 8.0;
       K_p_1 = 0.075;
       K_p_2 = 0.15;
     }
     
     if(end_row > finale_2){
     end_row = 0;
     end_line = 0;
     finale_1 = 0;
     init_pose = 0;
     line_tolerance = 2.0;
     ROS_INFO("New Row Starts!");
     finale_2 = end_row;
     }

     mini_goal = false;
     next_row_check = true;
     landmarks_pos.x.resize(6);
     landmarks_pos.y.resize(6);
     meas_pts.range.resize(4);
     meas_pts.bearing.resize(4);
     return true;
   }

int main(int argc, char **argv) {
  ros::init(argc, argv, "poly_nav_RANSAC");
  ros::NodeHandle n;
  ros::Rate r(10);

  // Subscribers
  ros::Subscriber scan_sub_test = n.subscribe("scan_filtered", 50, scanCallback);
  ros::Subscriber pose_sub_test = n.subscribe("robot_pose", 100, poseCallback);
  ros::Subscriber hpose_sub_test = n.subscribe("hokuyo_pose", 100, hposeCallback);
  ros::Subscriber lpose_sub_test = n.subscribe("line_pose", 100, lposeCallback);

  //Publishers
  marker_pub_1 = n.advertise<visualization_msgs::Marker>("line_marker_1", 10);
  marker_pub_2 = n.advertise<visualization_msgs::Marker>("line_marker_2", 10);
  marker_pub_3 = n.advertise<visualization_msgs::Marker>("final_line", 10);
  point_pub = n.advertise<thorvald_2d_nav::scan_detected_line>("measurement_points", 10);
  landmarks_pub = n.advertise<thorvald_2d_nav::landmarks>("landmark_points", 10);
  twist_gazebo = n.advertise<geometry_msgs::Twist>("/nav_vel", 100);  // control
  
  a_err = n.advertise<std_msgs::Float64>("/ang_err", 100);  // control

  // Service Client
  ros::ServiceClient client = n.serviceClient<thorvald_2d_nav::sub_goal>("/row_transition_mode");
  ros::ServiceClient client1 = n.serviceClient<thorvald_2d_nav::sub_goal>("/row_transition_mode_1");

  // Service Servers
  ros::ServiceServer service2 = n.advertiseService("/row_transition_end_1", row_transition);

  landmarks_pos.landmark_check = 0;
  landmarks_pos.row_number = 0;
  landmarks_pos.feature_no = 0;
  tf::StampedTransform transform;
  tf::TransformListener listener;
  meas_pts.range.resize(4);
  meas_pts.bearing.resize(4);
  landmarks_pos.x.resize(6);
  landmarks_pos.y.resize(6);
  landmarks_pos.x[0] = 0;
  landmarks_pos.y[0] = 0;
  landmarks_pos.x[1] = 0;
  landmarks_pos.y[1] = 0;
  landmarks_pos.x[2] = 0;
  landmarks_pos.y[2] = 0;
  landmarks_pos.x[3] = 0;
  landmarks_pos.y[3] = 0;
  landmarks_pos.x[4] = 0;
  landmarks_pos.y[4] = 0;
  landmarks_pos.x[5] = 0;
  landmarks_pos.y[5] = 0;
  prev_landmarks_pose.x.resize(6);
  prev_landmarks_pose.y.resize(6);
  prev_landmarks_pose.x[0] = 0;
  prev_landmarks_pose.x[2] = 0;

  while (ros::ok()) {
    ros::spinOnce(); 
 
if((init_pose==0)&&(thorvald_pose.position.x>0)){
finale = 0;
init_pose = 1;
}
  line_detect:
   if ((scan_msg_main.ranges.size() > 0) &&(pose_tmp==1) && (finale == 0)){  // check for new lines
   
   // RANSAC 
    line_detect_1:
      Line_detection_1(line_pose);
    line_detect_2:
      Line_detection_2(line_pose);
    
     left_line_trans[0].header.frame_id = "map";
     left_line_trans[1].header.frame_id = "map";
     right_line_trans[0].header.frame_id = "map";
     right_line_trans[1].header.frame_id = "map";

     try{
      listener.lookupTransform("map", "hokuyo", ros::Time(0), transform);
      transformStamped.header.stamp = ros::Time::now();
      transformStamped.header.frame_id = "map";
      transformStamped.child_frame_id = "hokuyo";
      transformStamped.transform.translation.x = transform.getOrigin().getX();
      transformStamped.transform.translation.y = transform.getOrigin().getY();
      transformStamped.transform.translation.z = transform.getOrigin().getZ();
      transformStamped.transform.rotation.x = transform.getRotation().getX();
      transformStamped.transform.rotation.y = transform.getRotation().getY();
      transformStamped.transform.rotation.z = transform.getRotation().getZ();
      transformStamped.transform.rotation.w = transform.getRotation().getW();
    }
    catch (tf::TransformException &ex)
    {
      // just continue on
    }

     if(std::isnan(transformStamped.transform.translation.x) || std::isnan(transformStamped.transform.translation.y)){
     goto the_end; 
     }

     tf2::doTransform(left_line_[0], left_line_trans[0], transformStamped);
     tf2::doTransform(left_line_[1], left_line_trans[1], transformStamped);
     tf2::doTransform(right_line_[0], right_line_trans[0], transformStamped);
     tf2::doTransform(right_line_[1], right_line_trans[1], transformStamped);
                
      if(std::fabs(left_line_trans[1].pose.position.x - left_line_trans[0].pose.position.x) < line_tolerance) {
      //ROS_INFO("Line 1 has to be re-detected!");
      landmarks_pos.landmark_check = landmarks_pos.landmark_check + 1;
      goto line_publish;
      }

      if(std::fabs(right_line_trans[1].pose.position.x - right_line_trans[0].pose.position.x) < line_tolerance){
      //ROS_INFO("Line 2 has to be re-detected!");
      landmarks_pos.landmark_check = landmarks_pos.landmark_check + 1;
      goto line_publish;
      }
  
      // Skip the one side of new feature   
      if((std::fabs(left_line_trans[0].pose.position.x - right_line_trans[0].pose.position.x) > 1.0) || (std::fabs(left_line_trans[1].pose.position.x - right_line_trans[1].pose.position.x) > 1.0)){ 
      goto line_publish; }

      // Skip the one side of new feature   
      if((std::fabs(right_line_trans[0].pose.position.x - left_line_trans[0].pose.position.x) > 1.0) || (std::fabs(right_line_trans[1].pose.position.x - left_line_trans[1].pose.position.x) > 1.0)){ 
      goto line_publish; }

      if((line_found_1==true)&&(line_found_2==true)){
      line_found_1 = false;
      line_found_2 = false;

      if(std::isnan(left_line_trans[0].pose.position.x)  || std::isnan(right_line_trans[0].pose.position.x)){
      goto the_end; 
      }

      if(std::isnan(left_line_trans[1].pose.position.x)  || std::isnan(right_line_trans[1].pose.position.x)){
      goto the_end; 
      }

      line_path[0].x = (left_line_trans[0].pose.position.x + right_line_trans[0].pose.position.x)/2;
      line_path[0].y = (left_line_trans[0].pose.position.y + right_line_trans[0].pose.position.y)/2;
      line_path[1].x = (left_line_trans[1].pose.position.x + right_line_trans[1].pose.position.x)/2;
      line_path[1].y = (left_line_trans[1].pose.position.y + right_line_trans[1].pose.position.y)/2;

               
      tf::Quaternion quat_g_1(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
      goal_yaw = tf::getYaw(quat_g_1);
   
     for(int q_1 = 0; q_1 <= 1; q_1++) {
        line_strip_1.points.push_back(left_line_trans[q_1].pose.position);
        line_strip_2.points.push_back(right_line_trans[q_1].pose.position);
        final_line.points.push_back(line_path[q_1]);
      }  // for loop end for storing points

      landmarks_pos.x[0] = left_line_trans[0].pose.position.x;
      landmarks_pos.y[0] = left_line_trans[0].pose.position.y;
      landmarks_pos.x[1] = left_line_trans[1].pose.position.x;
      landmarks_pos.y[1] = left_line_trans[1].pose.position.y;

      landmarks_pos.x[2] = right_line_trans[0].pose.position.x;
      landmarks_pos.y[2] = right_line_trans[0].pose.position.y;
      landmarks_pos.x[3] = right_line_trans[1].pose.position.x;
      landmarks_pos.y[3] = right_line_trans[1].pose.position.y;

      landmarks_pos.x[4] = line_path[0].x;
      landmarks_pos.y[4] = line_path[0].y;
      landmarks_pos.x[5] = line_path[1].x;
      landmarks_pos.y[5] = line_path[1].y;

      landmarks_pos.landmark_check = landmarks_pos.landmark_check + 1;

     if((row_transit_mode==0)||(row_transit_mode==2)){      
       if(((landmarks_pos.x[0] - prev_landmarks_pose.x[0]) < 2.0) && ((landmarks_pos.x[2] - prev_landmarks_pose.x[2]) < 2.0)) goto line_publish;
      }

      if(row_transit_mode==1){      
       if(((prev_landmarks_pose.x[0] - landmarks_pos.x[0]) < 2.0) && ((prev_landmarks_pose.x[2] - landmarks_pos.x[2]) < 2.0)) goto line_publish;
      }

      // New feature check
      if(init_pose == 1){
      landmarks_pos.feature_no = 0;
      init_pose = 2;
      }

      if((row_transit_mode==0)||(row_transit_mode==2)){      
       if(((landmarks_pos.x[0] - prev_landmarks_pose.x[0]) > 2.0) && ((landmarks_pos.x[2] - prev_landmarks_pose.x[2]) > 2.0)) {  
       landmarks_pos.feature_no = landmarks_pos.feature_no + 1;
       if (landmarks_pos.feature_no > 2){ 
       K_p_1 = 0.05;
       K_p_2 = 0.05;
       }
       }
      }

      if((row_transit_mode==1)){      
       if(((prev_landmarks_pose.x[0] - landmarks_pos.x[0]) > 2.0) && ((prev_landmarks_pose.x[2] - landmarks_pos.x[2]) > 2.0)) 
       ROS_INFO("In Middle Row");
       max_laser_range = 6.0;
       landmarks_pos.feature_no = landmarks_pos.feature_no + 1;
       if(landmarks_pos.feature_no > 1){ 
        K_p_1 = 0.05;
        K_p_2 = 0.05;
       }
      }

      line_strip_1.action = visualization_msgs::Marker::ADD;
      line_strip_1.pose.position.z = 0.8;
      line_strip_1.pose.orientation.w = 1.0;
      line_strip_1.type = visualization_msgs::Marker::LINE_STRIP;
      line_strip_1.lifetime = ros::Duration(0.1);
      line_strip_1.id = 1;
      line_strip_1.scale.x = 0.05;
      line_strip_1.color.b = 1.0;
      line_strip_1.color.a = 1.0;

      line_strip_2.action = visualization_msgs::Marker::ADD;
      line_strip_2.pose.position.z = 0.8;
      line_strip_2.pose.orientation.w = 1.0;
      line_strip_2.type = visualization_msgs::Marker::LINE_STRIP;
      line_strip_2.lifetime = ros::Duration(0.1);
      line_strip_2.id = 2;
      line_strip_2.scale.x = 0.05;
      line_strip_2.color.b = 1.0;
      line_strip_2.color.a = 1.0;

      final_line.action = visualization_msgs::Marker::ADD;
      final_line.pose.position.z = 0.0;
      final_line.pose.orientation.w = 1.0;
      final_line.type = visualization_msgs::Marker::LINE_STRIP;
      final_line.lifetime = ros::Duration(0.1);
      final_line.id = 3;
      final_line.scale.x = 0.1;
      final_line.color.r = 1.0;
      final_line.color.a = 1.0;

      meas_pts.meas_update = true;
      prev_landmarks_pose = landmarks_pos;
    
    } // New line found

    else{
      ROS_INFO("Line not detected by RANSAC!");
    }
  }  // check for new lines

    line_publish:  
   //---------------------------------------CONTROLLER--------------------------------------------//
      if ((landmarks_pos.landmark_check > line_count) && (lpose_tmp==1) ||((next_row_check == true))||(last_set_check==true)) {
        line_count = landmarks_pos.landmark_check;
        row_count = landmarks_pos.row_number;
    
       for (int i = 1; i <= Total_Points; i++) {
          if (landmarks_pos.x[5] > thorvald_pose.position.x) {
            Points[i].position.x = ((thorvald_pose.position.x) * (1 - (float(i) / Total_Points))) +((landmarks_pos.x[5] - tolerance) * (float(i) / Total_Points));
          } else {
            Points[i].position.x = ((thorvald_pose.position.x) * (1 - (float(i) / Total_Points))) +((landmarks_pos.x[5] + tolerance) * (float(i) / Total_Points));
          }
          Points[i].position.y = (thorvald_pose.position.y * (1 - (float(i) / Total_Points))) +((landmarks_pos.y[5]) * (float(i) / Total_Points));
        }

        mini_goal_pts.x = landmarks_pos.x[5];
        mini_goal_pts.y = landmarks_pos.y[5];
        mini_goal = true;
        //ROS_INFO("Assigned new Sub-goal");
        next_row_check = false;
        last_set_check = false;
      }

    if (mini_goal == true) {  // final min-goal check

        angular_velocity = control_law(line_pose);  // control law
        if (angular_velocity > 0.15) {
        angular_velocity = 0.1;
        ROS_INFO("Max Turning acheived");
        }
        if (angular_velocity < -0.15) {
        angular_velocity = -0.1;
        ROS_INFO("Max Turning acheived");
        }

        if(std::fabs(Points[Total_Points].position.x - (thorvald_pose.position.x)) <= 0.3) {
          mini_goal = false;
          est_twist.linear.x = 0;
          est_twist.angular.z = 0;
          finale = 1;
          end_row_check_1.request.counter = 1;         
          if (client1.call(end_row_check_1)) end_row_check.request.counter = 1; 
          if (client.call(end_row_check)) {std::cout<<"Reached End of Row_"<<landmarks_pos.row_number<< std::endl;}

          ROS_INFO("Final Mini-Goal Reached");

          line_strip_1 = empty_line_strip_1;
          line_strip_2 = empty_line_strip_2;
          final_line = empty_final_line;
          meas_pts = empty_meas_pts;
          landmarks_pos = empty_landmarks_pos;
          landmarks_pos.feature_no = 0;
          meas_pts.meas_update = false;
          landmarks_pos.landmark_check = 0; 
          }else{
          est_twist.linear.x = 0.25;  // SET TO ZERO UNTIL THE ABOVE ONES WORK
          est_twist.angular.z = angular_velocity;
          //est_twist.angular.z = 0.0;  // SET TO ZERO UNTIL THE ABOVE ONES WORK
        }

        stop:
        twist_gazebo.publish(est_twist);
      }  // final min-goal check 

    // Actual pho and alpha calculation (original measurements)    
    //tf::Quaternion Rotation_m ,Rotation_h_1, Rotation_h_2, Rotation_h_3, Rotation_h_4;
    //tf::Transform tf_hokuyo_map;
    //Rotation_m.setRPY(0, 0, yaw);

    // Landmark 1
   // Rotation_h_1.setRPY(0, 0, angle_1[first_l] + 2*M_PI);
   // tf_hokuyo_map.setRotation(Rotation_h_1 * Rotation_m);
   // meas_pts.bearing[0] = tf::getYaw(tf_hokuyo_map.getRotation());
    meas_pts.bearing[0] = normalizeangle(atan2(landmarks_pos.y[0],landmarks_pos.x[0])-yaw);
    //line_local[0] = sqrt(pow(left_line_trans[0].pose.position.y,2) + pow(left_line_trans[0].pose.position.x,2));
    line_local[0] = sqrt(pow(landmarks_pos.x[0],2) + pow(landmarks_pos.y[0],2));
    //meas_pts.range[0] = line_local[0] - thorvald_pose.position.x * cos(meas_pts.bearing[0]) - thorvald_pose.position.y * sin(meas_pts.bearing[0]);
    meas_pts.range[0] = line_local[0];
    //meas_pts.bearing[0] = meas_pts.bearing[0] + M_PI;

    // Landmark 2
    //Rotation_h_2.setRPY(0, 0, angle_1[second_l]+2*M_PI);
    //tf_hokuyo_map.setRotation(Rotation_h_2 * Rotation_m);
    //meas_pts.bearing[1] =  tf::getYaw(tf_hokuyo_map.getRotation());
    //meas_pts.bearing[1] = atan2(left_line_trans[1].pose.position.y, left_line_trans[1].pose.position.x)-yaw;
    meas_pts.bearing[1] = normalizeangle(atan2(landmarks_pos.y[1],landmarks_pos.x[1])-yaw);
    //line_local[1] = sqrt(pow(left_line_trans[1].pose.position.y,2) + pow(left_line_trans[1].pose.position.x,2));
    line_local[1] = sqrt(pow(landmarks_pos.x[1],2) + pow(landmarks_pos.y[1],2));
    //meas_pts.range[1] = line_local[1] - thorvald_pose.position.x * cos(meas_pts.bearing[1]) - thorvald_pose.position.y * sin(meas_pts.bearing[1]);
    meas_pts.range[1] = line_local[1];
    //meas_pts.bearing[1] = meas_pts.bearing[1] + M_PI;

    // Landmark 3
    //Rotation_h_3.setRPY(0, 0, angle_2[first_r]);
    //tf_hokuyo_map.setRotation(Rotation_h_3 * Rotation_m);
    //meas_pts.bearing[2] =  tf::getYaw(tf_hokuyo_map.getRotation());
    //meas_pts.bearing[2] = atan2(right_line_trans[0].pose.position.y, right_line_trans[0].pose.position.x)-yaw;
    meas_pts.bearing[2] = normalizeangle(atan2(landmarks_pos.y[2],landmarks_pos.x[2])-yaw);
    //line_local[2] =  sqrt(pow(right_line_trans[0].pose.position.y,2) + pow(right_line_trans[0].pose.position.x,2));
    line_local[2] = sqrt(pow(landmarks_pos.x[2],2) + pow(landmarks_pos.y[2],2));
    //meas_pts.range[2] = line_local[2] - thorvald_pose.position.x * cos(meas_pts.bearing[2]) - thorvald_pose.position.y * sin(meas_pts.bearing[2]);
    meas_pts.range[2] = line_local[2];

    // Landmark 4
    //Rotation_h_4.setRPY(0, 0, angle_2[second_r]);
    //tf_hokuyo_map.setRotation(Rotation_h_4 * Rotation_m);
    //meas_pts.bearing[3] =  tf::getYaw(tf_hokuyo_map.getRotation());
    //meas_pts.bearing[3] = atan2(right_line_trans[1].pose.position.y, right_line_trans[1].pose.position.x)-yaw;
    meas_pts.bearing[3] = normalizeangle(atan2(landmarks_pos.y[3],landmarks_pos.x[3])-yaw);
    //line_local[3] = sqrt(pow(right_line_trans[1].pose.position.y,2) + pow(right_line_trans[1].pose.position.x,2)); 
    line_local[3] = sqrt(pow(landmarks_pos.x[3],2) + pow(landmarks_pos.y[3],2));
    //meas_pts.range[3] = line_local[3] - thorvald_pose.position.x * cos(meas_pts.bearing[3]) - thorvald_pose.position.y * sin(meas_pts.bearing[3]);
    meas_pts.range[3] = line_local[3];

    line_strip_1.header.stamp = ros::Time::now();
    line_strip_2.header.stamp = ros::Time::now();
    final_line.header.stamp = ros::Time::now();
    meas_pts.header.stamp = ros::Time::now();
    landmarks_pos.header.stamp = ros::Time::now();

    marker_pub_1.publish(line_strip_1);
    marker_pub_2.publish(line_strip_2);
    marker_pub_3.publish(final_line);
    point_pub.publish(meas_pts);
    landmarks_pub.publish(landmarks_pos);

    meas_pts.header.frame_id = "map";
    line_strip_1.header.frame_id = "map";
    line_strip_2.header.frame_id = "map";
    landmarks_pos.header.frame_id = "map";
    final_line.header.frame_id = "map";

  the_end:
  r.sleep();
  }  // node shutdown
  return 0;
}
