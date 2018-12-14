#include "poly_nav_line_SLAM.h"

double dist_pt_l1[3], dist_pt_l2[3], result_1[2];
int min_itr[2], k, counter_sub = 0 ,counter_sub_1 = 0, counter_sub_2 = 0, counter_sub_3 = 0;
double slope, expectedBearing, expectedRange;
geometry_msgs::Pose line_div_1,line_div_2;
int stop_1 = 17.0;

// robot velocity data
void odometryvelCallback (const nav_msgs::Odometry::ConstPtr& odometry_vel){
robot_pose.twist.twist.linear = odometry_vel->twist.twist.linear;
robot_pose.twist.twist.angular = odometry_vel->twist.twist.angular;

counter_sub = 1;
}

// robot pose data
void odometryCallback (const geometry_msgs::Pose::ConstPtr& odomet){
thor_pose.position = odomet->position;
thor_pose.orientation = odomet->orientation;

tf::Quaternion quat(thor_pose.orientation.x,thor_pose.orientation.y, thor_pose.orientation.z, thor_pose.orientation.w);
quat = quat.normalize();
yaw = tf::getYaw(quat);

counter_sub_1 = 1;
}

//line point data
void linepointsCallback(const thorvald_2d_nav::scan_detected_line::ConstPtr& line_points){

actual_meas.range.resize(4);
actual_meas.bearing.resize(4);

if (line_points->meas_update == true){
for (int num = 0; num < total_points; num++){
actual_meas.range[num] = line_points->range[num];
actual_meas.bearing[num] = line_points->bearing[num];
actual_meas.meas_update = line_points->meas_update;
}}

counter_sub_2 = 1;
}

//line points 
void landmarksposeCallback (const thorvald_2d_nav::landmarks::ConstPtr& landmarks_msg)
{
landmarks_pose.x.resize(6);
landmarks_pose.y.resize(6);

if (landmarks_msg->landmark_check > 0){        
 for(int v=0; v<=5;v++){      
 landmarks_pose.x[v] = landmarks_msg->x[v];
 landmarks_pose.y[v] = landmarks_msg->y[v];
 }
landmarks_pose.landmark_check = landmarks_msg->landmark_check; 
landmarks_pose.row_number = landmarks_msg->row_number;
landmarks_pose.feature_no = landmarks_msg->feature_no;
}

counter_sub_3 = 1;
}

double normalizeangle(double bearing){
    if (bearing < -M_PI) {
        bearing += 2*M_PI;
    } else if (bearing > M_PI) {
        bearing -= 2*M_PI;
    }
}

//prediction step
void prediction_step(double dt, int f_no){

  vx = robot_pose.twist.twist.linear.x;
  vy = robot_pose.twist.twist.linear.y;
  vth = robot_pose.twist.twist.angular.z;

  // Odometry Compensation   
  mu(0,0) = mu(0,0) + (vx * cos(mu(2,0)))*dt;
  mu(1,0) = mu(1,0) + (vx * sin(mu(2,0)))*dt;
  // double si = vth*dt;
  //  mu(2,0) = mu(2,0) + (vx*(log(std::fabs(1/cos(si)))))/1.092;
  mu(2,0) = mu(2,0) + vth*dt;
  mu(2,0)= normalizeangle(mu(2,0));
std::cout << " " << vth << " " << mu(2,0) << std::endl;  
/*  for(int c=0;c<total_points; c++){
   mu((2*c)+3+(f_no*8),0) =  mu((2*c)+3+(f_no*8),0) - ((vx * cos(mu(2,0)) - vy * sin(mu(2,0)))*dt);
   mu((2*c)+4+(f_no*8),0) =  mu((2*c)+4+(f_no*8),0) - ((vx * sin(mu(2,0)) + vy * cos(mu(2,0)))*dt);
  }
*/

  MatrixXd Gt = MatrixXd(3,3);
  Gt << 1, 0, (-vx * sin(mu(2,0)) - vy * cos(mu(2,0)))*dt,
        0, 1, ( vx * cos(mu(2,0)) - vy * sin(mu(2,0)))*dt,
        0, 0,  1;                                       //why the last row is zero?

  R.topLeftCorner(3,3) << motion_noise, 0, 0,
                          0, motion_noise , 0,
                          0, 0,   motion_noise/10;      //why does it reduce ?

  cov.topLeftCorner(3,3) = Gt * cov.topLeftCorner(3,3) * Gt.transpose();
  cov.topRightCorner(3, 2*total_landmarks) = Gt * cov.topRightCorner(3, 2*total_landmarks);
  cov.bottomLeftCorner(2*total_landmarks, 3) = cov.topRightCorner(3, 2*total_landmarks).transpose();

  cov = cov + R; // Motion Noise

}

//correction step
bool correction_step(double dt, int f_no){

     // Landmarks observation
     for (int z = 0; z < total_points; z++){

      // Actual Measurements
      Z(2*z,0) = actual_meas.range[z];
      Z((2*z)+1,0) = actual_meas.bearing[z];
     
      // Transformation of Line from global co-ordinate into robot frame
      line_theta(z,0) = atan2(mu((2*z)+4+(f_no*8),0), mu((2*z)+3+(f_no*8),0))-yaw;
      line_pho(z,0) = sqrt( pow( mu((2*z)+3+(f_no*8),0),2)+ pow( mu((2*z)+4+(f_no*8),0),2));
      expectedZ((2*z),0) = line_pho(z,0);
      expectedZ((2*z)+1,0) = normalizeangle(line_theta(z,0));  
          
 //      std::cout << "z:" << z <<  "\n" << "Z(2*z,0):" << Z((2*z),0) <<  "\n"<< "expectedZ(2*z,0):" << expectedZ((2*z),0) <<  "\n" << std::endl;
      // std::cout << "z+1:" << z+1 <<  "\n" << "Z(2*z+1,0):" << Z((2*z)+1,0) <<  "\n"<< "expectedZ(2*z+1,0):" << expectedZ((2*z)+1,0) <<  "\n" << std::endl;
   // Compute the Jacobian H of the measurement function h wrt the landmark location (x,y,theta,pho,alpha)
       H((2*z),0) = 0;
       H((2*z)+1,0) = 0;
       H((2*z),1) = 0;
       H((2*z)+1,1) = 0;
       H((2*z),2) = 0;
       H((2*z)+1,2) = -1; 

       H((2*z),((2*z)+3+(f_no*8))) = mu(((2*z)+3+(f_no*8)),0)/line_pho(z,0);
       H((2*z)+1,((2*z)+3+(f_no*8))) = mu(((2*z)+4+(f_no*8)),0)/line_pho(z,0);;
       H((2*z),((2*z)+4+(f_no*8))) = -mu(((2*z)+4+(f_no*8)),0)/pow( mu((2*z)+3+(f_no*8),0),2)+ pow( mu((2*z)+3+(f_no*8),0),2);
       H((2*z)+1,((2*z)+4+(f_no*8))) = mu(((2*z)+3+(f_no*8)),0)/pow( mu((2*z)+3+(f_no*8),0),2)+ pow( mu((2*z)+3+(f_no*8),0),2);
       }	

       //expectedZ = H*mu;
	
       // Compute the Kalman gain
       K = cov*H.transpose()*(H*cov*H.transpose() + Q).inverse();

        // Compute the diference between the expected and recorded measurements.
       diff = Z - expectedZ;
       diff(1,0) = normalizeangle(diff(1,0));
       diff(3,0) = normalizeangle(diff(3,0));
       diff(5,0) = normalizeangle(diff(5,0));
       diff(7,0) = normalizeangle(diff(7,0));
      /* std::cout << "diff:" << diff(1,0) << "\n" << std::endl;
       std::cout << "diff:" << diff(3,0) << "\n" << std::endl;
       std::cout << "diff:" << diff(5,0) << "\n" << std::endl;
       std::cout << "diff:" << diff(7,0) << "\n" << std::endl;
      //ma_dist = diff.transpose()*(Q+Q).inverse()*diff; // Malabhonis distance check
*/
       if((std::fabs(diff(0,0))>th_dist)||(std::fabs(diff(2,0))>th_dist)||(std::fabs(diff(4,0))>th_dist)||(std::fabs(diff(6,0))>th_dist)) return false;
       if((std::fabs(diff(1,0))>0.05)||(std::fabs(diff(3,0))>0.05)||(std::fabs(diff(5,0))>0.05)||(std::fabs(diff(7,0))>0.05)) return false;

       // Finish the correction step by computing the new mu and sigma.
       mu = mu + K*diff;
       cov = (MatrixXd::Identity((2*total_landmarks+3),(2*total_landmarks+3))- K*H)*cov ;
       mu(2,0)= normalizeangle(mu(2,0));
} 

bool row_transition(thorvald_2d_nav::sub_goal::Request &req, thorvald_2d_nav::sub_goal::Response &res)
   {
     init_pose = req.counter;
     line_SLAM_status = true;
     row_transit_mode = row_transit_mode + req.counter;
  
     if(row_transit_mode == 1){
     stop_1 = 11.0;
     }

     if(row_transit_mode == 2) stop_1 = 16.0;
     f_n = 0;
     landmarks_pose.feature_no = 0;
     curr_f_n = 0;
     ROS_INFO("Line SLAM starts for new row");
     return true;
   }

bool row_end(thorvald_2d_nav::sub_goal::Request &req, thorvald_2d_nav::sub_goal::Response &res)
   {
     ROS_INFO("End of Line SLAM");
     line_SLAM_status = false;
     return true;
   }

int main(int argc, char** argv)
{
   ros::init(argc, argv, "poly_nav_line_SLAM");
   ros::NodeHandle n;
   ros::Rate r(10.0);

   // Subscribers
   ros::Subscriber landmarks_sub = n.subscribe("landmark_points", 100, landmarksposeCallback);
   ros::Subscriber odom_sub = n.subscribe("/odom_pose", 100, odometryCallback);
   ros::Subscriber odom_vel_sub = n.subscribe("/odometry/base_raw", 50, odometryvelCallback);
   ros::Subscriber point_sub = n.subscribe("measurement_points", 100, linepointsCallback);

   // Publishers
   ros::Publisher thorvald_pose_pub = n.advertise<nav_msgs::Odometry>("line_pose", 10);

   // Service Servers
   ros::ServiceServer service1 = n.advertiseService("/row_transition_mode_1", row_end);
   ros::ServiceServer service2 = n.advertiseService("/row_transition_end_2", row_transition);

   //robSigma(1,1) = 0.001;
   cov.topLeftCorner(3,3) = robSigma;
   cov.topRightCorner(3, (2*total_landmarks)) = robMapSigma;
   cov.bottomLeftCorner((2*total_landmarks), 3) = robMapSigma.transpose();
   cov.bottomRightCorner((2*total_landmarks), (2*total_landmarks)) = mapSigma;

   current_time = ros::Time::now();
   last_time = ros::Time::now();
   prev_meas_pts.range.resize(4);
   prev_meas_pts.bearing.resize(4);
   prev_landmarks_pose.x.resize(6);
   prev_landmarks_pose.y.resize(6);
   int i = 0;
   tf::TransformListener listener1;
   listener1.waitForTransform("map", "base_link", ros::Time(), ros::Duration(1.0));

   // Markers
   visualization_msgs::Marker landmark_strip_1, landmark_strip_2;

   while (ros::ok()){

   ros::spinOnce();	
   current_time = ros::Time::now();
   dt = (current_time-last_time).toSec();

  // if((landmarks_pose.landmark_check > 0) && (landmarks_pose.feature_no>0) && (line_SLAM_status==true) && (counter_sub == 1) && (counter_sub_2==1)&&(counter_sub_3==1)&&(actual_meas.meas_update == true)){
   if((counter_sub_1 == 1)){
   counter_sub_1 = 0;
   counter_sub_2 = 0;
   counter_sub_3 = 0;

   // initial state in the map frame
   if(init_pose==1){
 //  if(landmarks_pose.feature_no<2){
  
   tf::StampedTransform transform;
    try
    { 
      listener1.lookupTransform("odom", "base_link", ros::Time(0), transform);
    }catch (tf::TransformException &ex)
    {
      // just continue on
    }

  // mu(0,0) = transform.getOrigin().getX();
   mu(0,0) = thor_pose.position.x;
 //  mu(1,0) = transform.getOrigin().getY();
   mu(1,0) = thor_pose.position.y;
  // mu(2,0) = tf::getYaw(transform.getRotation());
   mu(2,0) = yaw;
  
   // std::cout  <<  "\n" << "yaw:" << mu(2,0) <<  "\n" << std::endl;
   landmarks_pose.feature_no = 0;
   curr_f_n = 0;
   init_pose = 2;
   }

/*   //std::cout << "feature_no:" << landmarks_pose.feature_no <<  "\n" << "curr_f_n:" << curr_f_n  <<  "\n" << std::endl;
   // Update Feature 
      if((landmarks_pose.feature_no>curr_f_n)){
      ROS_INFO("New Feature Observed");
      curr_f_n = landmarks_pose.feature_no; 
      f_n = landmarks_pose.feature_no - 1;   
      for(int p=0; p<total_points;p++){
       std::cout<<"landmarks_pose.feature_no:"<<landmarks_pose.feature_no<< " "<<"f_n:" << f_n <<" "<< "p:" << p <<  "\n" << "x:" << landmarks_pose.x[p] <<  "\n"<< "y:" << landmarks_pose.y[p] <<  "\n" << std::endl;
       mu((2*p)+3+(f_n*8),0) =  landmarks_pose.x[p];
       mu((2*p)+4+(f_n*8),0) =  landmarks_pose.y[p];
       }       
     }
  */  
  // if(f_n >= 1){ 

    // prediction step
   prediction_step(dt, f_n);
   
   // correction step    
   if((row_transit_mode == 0)||(row_transit_mode == 2)){
   if(mu(0,0) > stop_1) goto pose_estimate; }

   if(row_transit_mode == 1){
   if(mu(0,0) < stop_1) goto pose_estimate; }

  // correction_step(dt, f_n);

  // }

   pose_estimate:
    // Robot pose estimation
   thorvald_estimated_pose.pose.pose.position.x = mu(0,0);
   thorvald_estimated_pose.pose.pose.position.y = mu(1,0);
   geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(mu(2,0));
   thorvald_estimated_pose.pose.pose.orientation = q;
   thorvald_estimated_pose.header.stamp = ros::Time::now();
   thorvald_estimated_pose.header.frame_id = "map";
   thorvald_estimated_pose.child_frame_id = "base_link";

   thorvald_pose_pub.publish(thorvald_estimated_pose); 
  } // spin check

  last_time = current_time;
  r.sleep();
  } 
  return 0;
}
