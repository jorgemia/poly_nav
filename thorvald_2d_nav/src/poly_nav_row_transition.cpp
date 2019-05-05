#include "poly_nav_row_transition.h"

// Laser Scan data
void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
 num_ranges = scan_msg->ranges.size();
 scan_msg_main.header.stamp = scan_msg->header.stamp;
 scan_msg_main.ranges = scan_msg->ranges;
 scan_msg_main.angle_min = scan_msg->angle_min;
 scan_msg_main.angle_increment = scan_msg->angle_increment;
} // callback end

// Hokuyo pose data
void hposeCallback (const geometry_msgs::Pose::ConstPtr& hpose_msg){
hokuyo_pose.position = hpose_msg->position;
hokuyo_pose.orientation = hpose_msg->orientation; } 

// Laser Scan data
void robotposeCallback (const geometry_msgs::Pose::ConstPtr& pose_msg)
{
if((!std::isnan(pose_msg->position.x)) || (!std::isnan(pose_msg->position.y))){
thor_est.pose.position = pose_msg->position;
thor_est.pose.orientation = pose_msg->orientation;
tf::Quaternion quat(thor_est.pose.orientation.x,thor_est.pose.orientation.y, thor_est.pose.orientation.z, thor_est.pose.orientation.w);
quat = quat.normalize();
yaw = tf::getYaw(quat);
count_1 = 1;
}} // callback end

void create_markers(int id_no){
  marker_[id_no].header.stamp = ros::Time::now();
  marker_[id_no].header.frame_id = "map";
  marker_[id_no].ns = "poles";
  marker_[id_no].id = id_no;
  marker_[id_no].type = visualization_msgs::Marker::CYLINDER;
  marker_[id_no].action = visualization_msgs::Marker::ADD;
  marker_[id_no].pose.position.z = 0.90;
  marker_[id_no].pose.orientation.w = 1.0;
  marker_[id_no].scale.x = 0.3;
  marker_[id_no].scale.y = 0.3;
  if((id_no==0)||(id_no==1)||(id_no==2)) marker_[id_no].scale.z = 0.3;
  if((id_no==3)||(id_no==4)) marker_[id_no].scale.z = 0.5;
  marker_[id_no].color.a = 1.0; // Don't forget to set the alpha!
  if((id_no==0)||(id_no==1)||(id_no==2)) marker_[id_no].color.b = 1.0;
  if((id_no==3)||(id_no==4)) marker_[id_no].color.g = 1.0;
}

// Pole detection
void Pole_detection(sensor_msgs::LaserScan scan_msg_poles, double itr_begin_1, double itr_end_1, double itr_begin_2, double itr_end_2){

if(goal_found == false){

pole_detection:
double thershold = 0.3;
int initial_count_1 = 0, initial_count_2 = 0, unfit = 0, unfit_1 = 1;
int k = 0, q = 0, n = 0, unfit_2 = 0;
double x[num_ranges], y[num_ranges], angle[num_ranges];
double range_1[num_ranges], count_angle_1[num_ranges], count_angle_2[num_ranges];
double array_1[num_ranges], array_2[num_ranges], array_3[num_ranges]; 
double Current_x_1, Current_y_1, Sum_x_1, Sum_y_1, sum_k[num_ranges], current_range_[3], angle_[3];

//---------------------- FIRST MARKER ---------------------------//
        pole_redetect:
	for (int i = itr_begin_1; i <= itr_end_1; i++){
           if((scan_msg_poles.ranges[i] < min_range_right)){
            angle[initial_count_1] = scan_msg_poles.angle_min+i*scan_msg_poles.angle_increment;
            x[initial_count_1] = scan_msg_poles.ranges[i]*cos(angle[initial_count_1]);
            y[initial_count_1] = scan_msg_poles.ranges[i]*sin(angle[initial_count_1]);
            count_angle_1[initial_count_1] = i;
            initial_count_1 = initial_count_1 + 1;
           }
         } // for loop 	
          rand_pole_1_select:
          int random_count_1 = rand()% (initial_count_1-1);
          double current_itr_1 = count_angle_1[random_count_1];
          current_range_[0] = scan_msg_poles.ranges[current_itr_1];
          angle_[0] = scan_msg_poles.angle_min + current_itr_1*scan_msg_poles.angle_increment;
          Current_x_1 = x[random_count_1];
          Current_y_1 = y[random_count_1];
          if(std::fabs(Current_y_1>3.5)) goto rand_pole_1_select;
        for (int j = 0 ; j < initial_count_1; j++){
          double itr_loop_1 = count_angle_1[j];
  	  double obst_dist_1 = (current_range_[0] - scan_msg_poles.ranges[itr_loop_1]); // error calculation
          if ((thershold > fabs(obst_dist_1))&&(std::fabs(y[j]<3.5)) ){
              k = k + 1;
              sum_k[k] = count_angle_1[j];
             // Sum_x_1 = Sum_x_1 + x[j];
             // Sum_y_1 = Sum_y_1 + y[j];
          }
          else{
            array_1[unfit] = itr_loop_1;
            unfit = unfit + 1;
          }     
        }

//---------------------- SECOND MARKER ---------------------------//
  if(unfit>0){
  double sum_q[num_ranges];
  double Current_x_2, Current_y_2, Sum_x_2, Sum_y_2;
  
  rand_pole_2_select:
  int random_count_2 = rand()% (unfit-1);
  double current_itr_2 = array_1[random_count_2];
  Current_x_2 = x[random_count_2];
  Current_y_2 = y[random_count_2];
  if(std::fabs(Current_y_2>3.5)) goto rand_pole_2_select;
  current_range_[1] = scan_msg_poles.ranges[current_itr_2];
  angle_[1] = scan_msg_poles.angle_min + current_itr_2*scan_msg_poles.angle_increment;
  for (int p = 0 ; p < (unfit); p++){
   double itr_loop_2 = array_1[p];
   double obst_dist_2 = (current_range_[1] - scan_msg_poles.ranges[itr_loop_2]); // error calculation
   if ((thershold > fabs(obst_dist_2)) &&(std::fabs(y[p])<3.5)){
   sum_q[q] = array_1[p];
//   Sum_x_2 = Sum_x_2 + x[p];
//   Sum_y_2 = Sum_y_2 + y[p];   
   q = q + 1;
   }  
   else{
    array_2[unfit_1] = itr_loop_2; 
    unfit_1 = unfit_1 + 1;
    }         
   }  
  } 

  else{
  ROS_INFO("1st Set of Poles has to be re-detected");
  goto pole_redetect;
  }

//---------------------- THIRD MARKER ---------------------------//

  double sum_n[num_ranges];
  double x2[num_ranges], y2[num_ranges], angle2[num_ranges];
  double Current_x_3, Current_y_3, Sum_x_3, Sum_y_3;

     pole_redetect_left:
	for (int r = itr_begin_2; r <= itr_end_2; r++){
           if((scan_msg_poles.ranges[r] < min_range_left)){
            initial_count_2 = initial_count_2 + 1;
            count_angle_2[initial_count_2] = r;
            angle[initial_count_2] = scan_msg_poles.angle_min+r*scan_msg_poles.angle_increment;
            x2[initial_count_2] = scan_msg_poles.ranges[r]*cos(angle[initial_count_2]);
            y2[initial_count_2] = scan_msg_poles.ranges[r]*sin(angle[initial_count_2]);
           }
         } // for loop 	

          rand_pole_3_select:
          int random_count_3 = rand()% (initial_count_2);
          double current_itr_3 = count_angle_2[random_count_3];
          Current_x_3 = x2[random_count_3];
          Current_y_3 = y2[random_count_3];
          if(std::fabs(Current_y_3>3.0)) goto rand_pole_3_select;
          current_range_[2] = scan_msg_poles.ranges[current_itr_3];
          angle_[2] = scan_msg_poles.angle_min + current_itr_3*scan_msg_poles.angle_increment;

        for (int m = 1 ; m <= (initial_count_2); m++){
          double itr_loop_3 = count_angle_2[m];
  	  double obst_dist_3 = (current_range_[2] - scan_msg_poles.ranges[itr_loop_3]); // error calculation
          if ((thershold > fabs(obst_dist_3))&&(std::fabs(y[m]<2.2)) ){
             // Sum_x_3 = Sum_x_3 + x[m];
             // Sum_y_3 = Sum_y_3 + y[m];
              n = n + 1;
              sum_n[n] = count_angle_2[m];
          }       
        }

if(n == 0){
ROS_INFO("2nd Set of Poles has to be re-detected");
goto pole_redetect_left;
}
ROS_INFO("markers found");
//---------------------- GOAL ---------------------------// 
transformStamped.header.stamp = ros::Time::now();
transformStamped.header.frame_id = "map";
transformStamped.child_frame_id = "hokuyo";
transformStamped.transform.translation.x = hokuyo_pose.position.x;
transformStamped.transform.translation.y = hokuyo_pose.position.y;
transformStamped.transform.translation.z = hokuyo_pose.position.z;
transformStamped.transform.rotation.x = hokuyo_pose.orientation.x;
transformStamped.transform.rotation.y = hokuyo_pose.orientation.y;
transformStamped.transform.rotation.z = hokuyo_pose.orientation.z;
transformStamped.transform.rotation.w = hokuyo_pose.orientation.w;
if((std::isnan(transformStamped.transform.translation.x)) || (std::isnan(transformStamped.transform.translation.y))) goto pole_detection;

for(int po=0;po<3;po++){
pole_[po].header.frame_id = "hokuyo";
pole_[po].pose.position.x = current_range_[po]*cos(angle_[po]);
pole_[po].pose.position.y = current_range_[po]*sin(angle_[po]);
tf2::doTransform(pole_[po], pole_trans_[po], transformStamped); // transform into map frame
marker_[po].pose.position = pole_trans_[po].pose.position;
create_markers(po); // create the markers
}

 if(unfit>0){
  if(marker_[0].pose.position.y > marker_[1].pose.position.y) nearest_pole.position = marker_[0].pose.position;
  else nearest_pole.position = marker_[1].pose.position;

   if(row_transit_mode==1){ 
  marker_[3].pose.position.x = ((marker_[2].pose.position.x + nearest_pole.position.x)/2) + 2.5;
  marker_[4].pose.position.x = ((marker_[0].pose.position.x + marker_[1].pose.position.x)/2) + 2.5;
  marker_[3].pose.position.y = ((marker_[2].pose.position.y + nearest_pole.position.y)/2)-0.1;
  marker_[4].pose.position.y = ((marker_[0].pose.position.y + marker_[1].pose.position.y)/2)-0.1;
 }
  else{ 
  marker_[3].pose.position.x = ((marker_[2].pose.position.x + nearest_pole.position.x)/2) - 2.5;
  marker_[4].pose.position.x = ((marker_[0].pose.position.x + marker_[1].pose.position.x)/2) - 2.5; }
  marker_[3].pose.position.y = ((marker_[2].pose.position.y + nearest_pole.position.y)/2)+0.2;
  marker_[4].pose.position.y = ((marker_[0].pose.position.y + marker_[1].pose.position.y)/2)+0.2;
 
  create_markers(3); // create the goal marker 1
  create_markers(4); // create the goal marker 2
  goal_pt[0].position = marker_[3].pose.position;
  goal_pt[1].position = marker_[4].pose.position;
  goal_found = true;
  }
 }
}

double normalizeangle(double bearing){
 if(bearing < -M_PI) bearing += 2*M_PI;
 if(bearing > M_PI) bearing -= 2*M_PI;
}

double pure_pursuit(double obs_range, double obs_bearing){
double robotlength = 1.0;
double steeringAngle = normalizeangle(atan(2*robotlength*sin(obs_bearing))/obs_range);
return steeringAngle;
}

bool change_row(thorvald_2d_nav::sub_goal::Request &req, thorvald_2d_nav::sub_goal::Response &res)
   {
     ROS_INFO("transition service on time"); 
     if (row_transit_mode < 2)row_transit_mode = row_transit_mode + req.counter; // Change only one row at the moment
     if(row_transit_mode == 1) turn_side = 1;
     if(row_transit_mode == 2) turn_side = 2;
     turning_180=false;
     turn_90=false;  
     return true;
   }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "poly_nav_row_transition");
  ros::NodeHandle n;
  //ros::Rate r(1.0);

  // Subscribers
  ros::Subscriber scan_sub_test = n.subscribe("scan", 100, scanCallback);
  ros::Subscriber pose_sub = n.subscribe("robot_pose", 100, robotposeCallback);
  ros::Subscriber hpose_sub = n.subscribe("hokuyo_pose", 100, hposeCallback); // tf map to hokuyo
  
  //Publishers
  ros::Publisher twist_gazebo = n.advertise<geometry_msgs::Twist>( "/nav_vel", 100);
  ros::Publisher vis_pub_1 = n.advertise<visualization_msgs::Marker>( "pole_marker_1", 1 );
  ros::Publisher vis_pub_2 = n.advertise<visualization_msgs::Marker>( "pole_marker_2", 1 );
  ros::Publisher vis_pub_3 = n.advertise<visualization_msgs::Marker>( "pole_marker_3", 1 );
  ros::Publisher vis_pub_4 = n.advertise<visualization_msgs::Marker>( "goal_marker_1", 1 );
  ros::Publisher vis_pub_5 = n.advertise<visualization_msgs::Marker>( "goal_marker_2", 1 );
  ros::Publisher a_err = n.advertise<std_msgs::Float64>( "/ang_err_rt", 1);

  // Service Servers
  ros::ServiceServer service = n.advertiseService("/row_transition_mode", change_row);

  // Service Client
  ros::ServiceClient client = n.serviceClient<thorvald_2d_nav::sub_goal>("/row_transition_end_1");
  ros::ServiceClient client1 = n.serviceClient<thorvald_2d_nav::sub_goal>("/row_transition_end_2");

  while (ros::ok()){
  ros::spinOnce();

  if((scan_msg_main.ranges.size() > 0) && (count_1 == 1) && (row_transit_mode>0) && (row_transit_mode == row_transit)){ // scan check  

   if(turn_side == 1){
   min_itr_1 = 0;
   max_itr_1 = 359;
   min_itr_2 = 360;
   max_itr_2 = 719; }

   if(turn_side == 2){
   min_itr_1 = 360;
   max_itr_1 = 719;
   min_itr_2 = 0;
   max_itr_2 = 359; }
  
  Pole_detection(scan_msg_main, min_itr_1 , max_itr_1, min_itr_2 , max_itr_2); // pole detection function

  if(goal_found == true){ //goal check
   goal_range = sqrt(pow((goal_pt[goal_transit-1].position.y - thor_est.pose.position.y),2) + pow((goal_pt[goal_transit-1].position.x - thor_est.pose.position.x),2));
   goal_bearing = normalizeangle(atan2((goal_pt[goal_transit-1].position.y-thor_est.pose.position.y),(goal_pt[goal_transit-1].position.x - thor_est.pose.position.x))-yaw);
   
   ang_error.data = goal_bearing;

   
   if((goal_range < min_goal_range)){
    est_twist.linear.x = 0.0; 
    est_twist.angular.z = 0.0;
    in_place_turn = true; 
    }

  if(in_place_turn==true){
   switch(turn_side){ 
    case RIGHT:
    if(yaw>3.10) yaw = -yaw;
    
    if(goal_transit==1){
     if(std::fabs(1.57+yaw)>0.1){
     est_twist.angular.z = -0.2; 
    est_twist.linear.x = 0.0; 
     turn_90=false;  
     }
     else{
     turn_90 = true;
     goal_transit = goal_transit + 1;
     est_twist.angular.z = 0.0;
     est_twist.linear.x = 0.0; 
     in_place_turn = false; 
     ros::Duration(2.0).sleep(); // sleep for half a second
     }
    }
   
     else if((goal_transit==2)){
     if(std::fabs(3.14+yaw)>0.1){
     est_twist.angular.z = -0.2; 
     est_twist.linear.x = 0.0; 
     min_goal_range = 0.2;
     turn_90=false;  
     turning_180=true;
     }
     else{
     turn_90 = true;
     goal_transit = goal_transit + 1;
     est_twist.angular.z = 0.0;
     est_twist.linear.x = 0.0; 
     in_place_turn = false; 
     ros::Duration(2.0).sleep(); // sleep for half a second
     }
    }
    // std::cout << "goal_range:" << goal_range << "\n" << "yaw:" << yaw << std::endl;
    break;

    case LEFT:
    if(goal_transit==1) goal_transit1 = 1;
    if(goal_transit==2) goal_transit1 = 0.1;
    if(yaw>3.10) yaw = -yaw;

    if(goal_transit==1){
     if(std::fabs(1.57+yaw)>0.05){
     est_twist.angular.z = 0.2; 
     est_twist.linear.x = 0.0; 
     turn_90=false;  
     }
     else{
     turn_90 = true;
     goal_transit = goal_transit + 1;
     est_twist.angular.z = 0.0;
     in_place_turn = false; 
     ros::Duration(2.0).sleep(); // sleep for half a second
     }
    }
    if(goal_transit==2){
     if(yaw<-0.1){
     est_twist.angular.z = 0.1; 
     est_twist.linear.x = 0.0; 
     turn_90=false;  
     }
     else{
     turn_90 = true;
     goal_transit = goal_transit + 1;
     est_twist.angular.z = 0.0;
     in_place_turn = false; 
     ros::Duration(2.0).sleep(); // sleep for half a second
     break;
    }
   }
  break;
   }
  }
  

   if((turning_180==false)&&(in_place_turn==false)){
   angular_velocity = pure_pursuit(goal_range, goal_bearing); //pure pursuit controller
   if (angular_velocity>0.2) angular_velocity = 0.2;
   if (angular_velocity<-0.2) angular_velocity = -0.2;
   a_err.publish(ang_error);

   if((goal_range < max_goal_range)) est_twist.linear.x = 0.3; 
   else est_twist.linear.x = 0;
   est_twist.angular.z = angular_velocity; 
    }
    

   // Service for end of row transition
   the_end:
   if ((turn_90 = true) && (goal_transit==3)){
     end_row_transit.request.counter = 1;
     end_row_transit_1.request.counter = 1;
     est_twist.angular.z = 0;
     row_transit = 1 + row_transit_mode;
     for(int em=0;em<5;em++) marker_[em] = empty_marker_[em];

     switch(turn_side){
      case RIGHT: 
      turn_side = 2;
      ROS_INFO("Turned Right");
      break;

      case LEFT:
      turn_side = 1;
      ROS_INFO("Turned Left");
      break;
     }

     if (client.call(end_row_transit)){ 
     ROS_INFO("End of the row transition");
     goal_transit = 1;
     goal_found = false;
     } 
     if (client1.call(end_row_transit_1)){
     ROS_INFO("End of the row transition_1");
       } 
   } 

  stopover:  // publish the markers
  vis_pub_1.publish(marker_[0]);
  vis_pub_2.publish(marker_[1]);
  vis_pub_3.publish(marker_[2]);
  vis_pub_4.publish(marker_[3]);
  vis_pub_5.publish(marker_[4]);
  twist_gazebo.publish(est_twist);
   } // goal check 
  } // scan check

     //r.sleep(); 
  } // node shutdown
 return 0;
} // main loop end
