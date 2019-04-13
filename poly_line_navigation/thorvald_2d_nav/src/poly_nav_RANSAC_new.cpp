#include "poly_nav_RANSAC_new.h"

Ransac::Ransac(){
 
  if(one_time==0){
   for(int re=0;re<=3;re++){ 
   line_[n].x = 0;
   line_[n].y = 0;  
   line_trans_[n].x = 0;
   line_trans_[n].y = 0;
   prev_line_trans_[n].x = 0;
   prev_line_trans_[n].y = 0; } 
   one_time = 1;
  }

  // Subscribers
  scan_sub = nh_.subscribe("scan_filtered", 100, &Ransac::scanCallback, this);
  pose_sub = nh_.subscribe("robot_pose", 100, &Ransac::poseCallback, this); // tf map to base link
  hpose_sub = nh_.subscribe("hokuyo_pose", 100, &Ransac::hposeCallback, this); // tf map to hokuyo
  lpose_sub = nh_.subscribe("line_pose", 100, &Ransac::lposeCallback, this); // EKF pose
 
  // Publishers
  marker_pub_1 = nh_.advertise<visualization_msgs::Marker>("line_marker_1", 10);
  marker_pub_2 = nh_.advertise<visualization_msgs::Marker>("line_marker_2", 10);
  marker_pub_3 = nh_.advertise<visualization_msgs::Marker>("final_line", 10);
  point_pub = nh_.advertise<thorvald_2d_nav::scan_detected_line>("measurement_points", 10);
  landmarks_pub = nh_.advertise<thorvald_2d_nav::landmarks>("landmark_points", 10);
  twist_gazebo = nh_.advertise<geometry_msgs::Twist>("nav_vel", 100);  // control
  a_err = nh_.advertise<std_msgs::Float64>("ang_err", 100);  

  // ROS Service Clients
  client = nh_.serviceClient<thorvald_2d_nav::sub_goal>("/row_transition_mode");;
  // client1 = nh_.serviceClient<thorvald_2d_nav::sub_goal>("/row_transition_mode_1");

  // ROS Service Server
  service_obj = nh_.advertiseService("/row_transition_end_1", &Ransac::row_transition, this);
}

void Ransac::initialize(){
transformStamped.header.stamp = ros::Time::now();
transformStamped.header.frame_id = "map";
transformStamped.child_frame_id = "hokuyo";
meas_pts.range.resize(4);
meas_pts.bearing.resize(4);
prev_meas_pts.range.resize(4);
prev_meas_pts.bearing.resize(4);
landmarks_pose.x.resize(6);
landmarks_pose.y.resize(6);
prev_landmarks_pose.x.resize(6);
prev_landmarks_pose.y.resize(6);
landmarks_pose.landmark_check = 0;
landmarks_pose.row_number = 0;
landmarks_pose.feature_no = 0;
meas_pts.meas_update = false;
prev_meas_pts.range[0] = 0;
prev_meas_pts.bearing[0] = 0;
for(int lp=0;lp<=5;lp++){
landmarks_pose.x[lp] = 0;
landmarks_pose.y[lp] = 0;
}

}

// Laser Scan data
void Ransac::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {

  num_ranges = scan_msg->ranges.size();
  scan_msg_main.header.stamp = scan_msg->header.stamp;
  scan_msg_main.ranges = scan_msg->ranges;
  scan_msg_main.angle_min = scan_msg->angle_min;
  scan_msg_main.angle_increment = scan_msg->angle_increment;

  // Line detection
  if((scan_msg_main.ranges.size() > 0)&&(both_lines_found==false)&&(row_follow_mode==true)){

  line_detection: 
  for (int ki = 0; ki <= k; ki++){  // Number of iterations

    // x, y, theta calculation for all scans
    for (int i_1 = 0; i_1 <= num_ranges; i_1++) {
     theta[i_1] = scan_msg_main.angle_min + i_1 * scan_msg_main.angle_increment;
     x[i_1] = scan_msg_main.ranges[i_1] * cos(theta[i_1]);
     y[i_1] = scan_msg_main.ranges[i_1] * sin(theta[i_1]); 

    if (!std::isnan(x[i_1]) && !std::isnan(y[i_1]) && (scan_msg_main.ranges[i_1] < max_laser_range) && (std::fabs(y[i_1])<2.0)){
      scan_pts_index_array[s_pt] = i_1;
      s_pt = s_pt + 1;
     }  // storing the ith value with pre-conditions
    }  // storing the x,y values (end for loop)

    s_pt = s_pt - 1;
    if(s_pt > d){ // min number of inliers check

    random_selection:
    line_1_pt_1 = rand() % (s_pt); // random selection for point 1
    line_1_pt_2 = rand() % (s_pt); // random selection for point 2
    aIndex_1 = scan_pts_index_array[line_1_pt_1];
    bIndex_1 = scan_pts_index_array[line_1_pt_2];
     if ((aIndex_1 != bIndex_1) && (y[aIndex_1] != 0) && (x[aIndex_1] != 0) && (y[bIndex_1] != 0) && (x[bIndex_1] != 0)){ // Line Model parameters
      model.m = (y[bIndex_1] - y[aIndex_1]) / (x[bIndex_1] - x[aIndex_1]);
      model.b = y[aIndex_1] - model.m * x[aIndex_1];
     }
     else  goto random_selection; 

       for (int q = 0; q <=(s_pt); q++) {
        index_no = scan_pts_index_array[q];
        error = y[index_no] - (model.m * x[index_no] + model.b);  // error calculation
        if((thershold > std::fabs(error)) && (error != 0)){  // check for threshold
         current_best_inlier_set[total_pts].real_x = x[index_no];
         current_best_inlier_set[total_pts].real_y = y[index_no];
         total_pts = total_pts + 1;
        }
       }  // for loop end

      total_pts = total_pts - 1;

      if((total_pts > d) && (total_pts > best_no_inliers)){  // selecting the inliers with max of points
         if (scan_msg_main.ranges[aIndex_1] < scan_msg_main.ranges[bIndex_1]){ // nearest points in the line
          first_pt = aIndex_1;
          second_pt = bIndex_1;
         }else{
          first_pt = bIndex_1;
          second_pt = aIndex_1;
         }

      // Conditions Check
        // Condition 1 (Distance between two poles check)
        if(std::fabs(x[second_pt] - x[first_pt])< 1.0) goto skip_line; 

        // Condition 2 (Angle between two poles check)
        if(std::fabs(atan2(y[second_pt]-y[first_pt],x[second_pt]-x[first_pt]))> 0.175) goto skip_line; 

        // Condition 3 (Same line detection check)
        if(n==2){ if((std::fabs(y[first_pt]-line_[0].y)< 1.0)||(std::fabs(y[second_pt]-line_[1].y)< 1.0)) goto skip_line; }

        // Condition 4 (Avoid detecting best line in next rows)
        if((std::fabs(y[first_pt])> 1.75)||(std::fabs(y[second_pt])> 1.75)) goto skip_line;

        line_[n].x = x[first_pt];
        line_[n].y = y[first_pt];
        line_[n+1].x = x[second_pt];
        line_[n+1].y = y[second_pt];
        if(n<2) one_line_found = true; 
        if(n==2) both_lines_found = true;
        
        skip_line:
        best_no_inliers = total_pts;
     }
    } // min inliers check

   s_pt = 0;
   total_pts = 0;
   best_no_inliers = 0;
   }// End of iterations
 
   if((one_line_found==true)&&(n<2)){ n = 2; goto line_detection; }
   n = 0;
   // if(both_lines_found==true) n = 0; 
  } // End of line detection
 } // End of callback

// AMCL pose data
void Ransac::poseCallback (const geometry_msgs::Pose::ConstPtr& pose_msg){
 if((!std::isnan(pose_msg->position.x)) || (!std::isnan(pose_msg->position.y))){
 thorvald_pose.position = pose_msg->position;
 thorvald_pose.orientation = pose_msg->orientation;
 tf::Quaternion quat(thorvald_pose.orientation.x,thorvald_pose.orientation.y, thorvald_pose.orientation.z, thorvald_pose.orientation.w);
 quat = quat.normalize();
 yaw = tf::getYaw(quat);
 }
}

// Hokuyo pose data
void Ransac::hposeCallback (const geometry_msgs::Pose::ConstPtr& hpose_msg){
hokuyo_pose.position = hpose_msg->position;
hokuyo_pose.orientation = hpose_msg->orientation;
} 

// EKF line pose data
void Ransac::lposeCallback (const nav_msgs::Odometry::ConstPtr& lpose_msg){
line_pose.position = lpose_msg->pose.pose.position;
line_pose.orientation = lpose_msg->pose.pose.orientation;
} 

// Normalize the bearing
double Ransac::normalizeangle(double bearing){
  if (bearing < -M_PI) {
    bearing += 2 * M_PI;
  } else if (bearing > M_PI) {
    bearing -= 2 * M_PI;
  }
}

bool Ransac::row_transition(thorvald_2d_nav::sub_goal::Request &req, thorvald_2d_nav::sub_goal::Response &res)
   {
     ROS_INFO("Next Row");
     row_transit_mode = row_transit_mode + 1;
     landmarks_pose.row_number = landmarks_pose.row_number + req.counter;
     landmarks_pose.feature_no = 0;
     row_follow_mode = true;
     one_time = 0;
     line_count = 0;
     landmarks_pose.x.resize(6);
     landmarks_pose.y.resize(6);
     meas_pts.range.resize(4);
     meas_pts.bearing.resize(4);
     return true;
   }

void Ransac::create_markers(int id_no){
      line_strip[id_no].header.frame_id = "map";
      line_strip[id_no].header.stamp = ros::Time::now();
      line_strip[id_no].action = visualization_msgs::Marker::ADD;
      line_strip[id_no].pose.position.z = 0.0;
      line_strip[id_no].type = visualization_msgs::Marker::LINE_STRIP;
      line_strip[id_no].lifetime = ros::Duration(0.1);
      line_strip[id_no].scale.x = 0.05; 
      if((id_no==0)||(id_no==1)) line_strip[id_no].color.b = 1.0;
      if(id_no==2) line_strip[id_no].color.r = 1.0;
      line_strip[id_no].color.a = 1.0;
      line_strip[id_no].points.resize(2);
}

void Ransac::calculate_measurements(){

  if((both_lines_found==true)){
  transformStamped.transform.translation.x = hokuyo_pose.position.x;
  transformStamped.transform.translation.y = hokuyo_pose.position.y;
  transformStamped.transform.translation.z = hokuyo_pose.position.z;
  transformStamped.transform.rotation.x = hokuyo_pose.orientation.x;
  transformStamped.transform.rotation.y = hokuyo_pose.orientation.y;
  transformStamped.transform.rotation.z = hokuyo_pose.orientation.z;
  transformStamped.transform.rotation.w = hokuyo_pose.orientation.w;

  for(int ic=0; ic<4; ic++){ // Landmarks Positions
  tf2::doTransform(line_[ic], line_trans_[ic], transformStamped);  // Switch from Hokuyo frame to Map Frame
  landmarks_pose.x[ic] = line_trans_[ic].x;
  landmarks_pose.y[ic] = line_trans_[ic].y;
  }

  landmarks_pose.x[4] = (landmarks_pose.x[0] + landmarks_pose.x[2])/2;
  landmarks_pose.y[4] = (landmarks_pose.y[0] + landmarks_pose.y[2])/2;
  landmarks_pose.x[5] = (landmarks_pose.x[1] + landmarks_pose.x[3])/2;
  landmarks_pose.y[5] = (landmarks_pose.y[1] + landmarks_pose.y[3])/2;

  for(int re=0;re<=3;re++) { prev_line_trans_[re] = line_trans_[re]; // Update Previous line detection
  prev_meas_pts.range[re] = meas_pts.range[re]; 
  prev_meas_pts.bearing[re] = meas_pts.bearing[re];} // Update Previous Measurements
  for(int re=0;re<=5;re++){ prev_landmarks_pose.x[re] = landmarks_pose.x[re]; 
                            prev_landmarks_pose.y[re] = landmarks_pose.y[re];} // Update Previous Landmarks Positions

  } // both the lines found check

  else{ // Use Previous best line fit if not found
  for(int pl=0;pl<=3;pl++) { line_trans_[pl] = prev_line_trans_[pl]; 
  meas_pts.range[pl] = prev_meas_pts.range[pl]; 
  meas_pts.bearing[pl] = prev_meas_pts.bearing[pl];} // Update Previous Measurements
  for(int pl=0;pl<=5;pl++){ landmarks_pose.x[pl] = prev_landmarks_pose.x[pl]; 
                            landmarks_pose.y[pl] = prev_landmarks_pose.y[pl];} // Update Previous Landmarks Positions
  }

    for(int i=0; i<=2; i++) { 
    create_markers(i); // function for creating the line markers
     if((i==0)||(i==1)){
      line_strip[i].points[0] = line_trans_[2*i];
      line_strip[i].points[1] = line_trans_[(2*i)+1];
      if(i==0) marker_pub_1.publish(line_strip[i]); // publish the detected line 1
      if(i==1) marker_pub_2.publish(line_strip[i]); // publish the detected line 2 
     }
     if(i==2){ // desired trajectory
     line_strip[i].points[0].x = (line_strip[0].points[0].x + line_strip[1].points[0].x)/2;
     line_strip[i].points[0].y = (line_strip[0].points[0].y + line_strip[1].points[0].y)/2;
     line_strip[i].points[1].x = (line_strip[0].points[1].x + line_strip[1].points[1].x)/2;
     line_strip[i].points[1].y = (line_strip[0].points[1].y + line_strip[1].points[1].y)/2;
     marker_pub_3.publish(line_strip[i]); // publish the trajectory 
     }
    } 

  for(int pt=0; pt<4; pt++){ // Actual measurements (pho and alpha calculation) 
    meas_pts.bearing[pt] = normalizeangle(atan2(landmarks_pose.y[pt],landmarks_pose.x[pt])-yaw);
    meas_pts.range[pt] = sqrt(pow(landmarks_pose.x[pt],2) + pow(landmarks_pose.y[pt],2));
  }

    // Feature Update
    d_line = sqrt(w_p*pow((prev_meas_pts.range[0]-meas_pts.range[0]),2)+w_b*pow((prev_meas_pts.bearing[0]-meas_pts.bearing[0]),2)); // Data Association
    if(d_line>d_ther) landmarks_pose.feature_no = landmarks_pose.feature_no + 1;
    landmarks_pose.landmark_check = landmarks_pose.landmark_check + 1;
}

void Ransac::controller(){

  if ((landmarks_pose.landmark_check > line_count)) {
    line_count = landmarks_pose.landmark_check;
    row_count = landmarks_pose.row_number;

    for (int i = 1; i <= Total_Points; i++) {
     if(landmarks_pose.x[5] > thorvald_pose.position.x) Points[i].position.x = ((thorvald_pose.position.x)*(1-(float(i) / Total_Points))) + ((landmarks_pose.x[5]-tolerance)*(float(i)/Total_Points));
     else Points[i].position.x = ((thorvald_pose.position.x) * (1 - (float(i) / Total_Points))) +((landmarks_pose.x[5] + tolerance) * (float(i) / Total_Points));
     Points[i].position.y = (thorvald_pose.position.y * (1 - (float(i) / Total_Points))) +((landmarks_pose.y[5]) * (float(i) / Total_Points));
    }
   }

   if(std::fabs(landmarks_pose.x[4]-thorvald_pose.position.x)>1.5){
   mini_goal_pts.x = landmarks_pose.x[4];
   mini_goal_pts.y = landmarks_pose.y[4];
   }
   else{
   mini_goal_pts.x = landmarks_pose.x[5];
   mini_goal_pts.y = landmarks_pose.y[5]; }

        // calculation of error
        q_x = mini_goal_pts.x - thorvald_pose.position.x;
        q_y = mini_goal_pts.y - thorvald_pose.position.y; 

        // range, bearing
        position_error = sqrt(pow(q_x, 2) + pow(q_y, 2));
        angular_error.data = normalizeangle(atan2(q_y, q_x) - yaw);
        a_err.publish(angular_error); // publish the angular error

        angular_velocity = normalizeangle(atan2(2*1.05*sin(angular_error.data),position_error)); // Pure Pursuit Controller

        if(std::isnan(angular_velocity)) angular_velocity = 0;
        if (angular_velocity > 0.05) angular_velocity = 0.05;
        if (angular_velocity < -0.05) angular_velocity = -0.05;

        if(std::fabs(Points[Total_Points].position.x - thorvald_pose.position.x) <= 0.3) {
          est_twist.linear.x = 0; // stop the robot
          est_twist.angular.z = 0;

          end_row_check.request.counter = 1;  // call the service for row transition mode
          //if (client1.call(end_row_check_1)) end_row_check.request.counter = 1;
          if (client.call(end_row_check)) {std::cout<<"Reached End of Row_"<< row_transit_mode<< std::endl;}

          ROS_INFO("Final Mini-Goal Reached");

          for (int els=0;els<3;els++){ line_strip[els] = empty_line_strip[els]; }
          meas_pts = empty_meas_pts;
          landmarks_pose = empty_landmarks_pose;
          prev_landmarks_pose = empty_landmarks_pose;
          landmarks_pose.feature_no = 0;
          meas_pts.meas_update = false;
          row_follow_mode = false;
          landmarks_pose.landmark_check = 0; 
          }else{
          est_twist.linear.x = 0.0;  // SET TO ZERO UNTIL THE ABOVE ONES WORK
          est_twist.angular.z = 0.0;
          //est_twist.angular.z = angular_velocity;
        }

        stop:
        twist_gazebo.publish(est_twist);
        both_lines_found = false;
}

void Ransac::move(){
  initialize();
  ros::Rate r(10);

  while(ros::ok()){
  ros::spinOnce();

  if(row_follow_mode == true){ // row follow mode
  calculate_measurements(); // obtaining the measurements data
  controller(); // low-level controller
  } // row follow mode

  meas_pts.meas_update = true;
  meas_pts.header.stamp = ros::Time::now();
  landmarks_pose.header.stamp = ros::Time::now();
  point_pub.publish(meas_pts); // publish measurements pho and alpha
  landmarks_pub.publish(landmarks_pose); // publish landmark x and y

  r.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "poly_nav_RANSAC_new");

  Ransac ransac;
  ransac.move();

  return 0;
};



