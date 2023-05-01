#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h> 
#include <time.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <math.h>
#include <ros/duration.h>
#include <chrono>

using namespace std;
string filename_;
double start_time; 

string date_filename()
{
  auto now = std::chrono::system_clock::now();
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  struct tm *parts = std::localtime(&now_c);
  //Assign
  std::string filename=               std::to_string(parts->tm_mday) //Set Year
    + "_" + std::to_string(parts->tm_mon+1) //Set Month
    + "_" + std::to_string(parts->tm_year-100) //Set day
    + "_" + std::to_string(parts->tm_hour) //Set hour
    + "min" + std::to_string(parts->tm_min) ;
  return filename;	

}


void copter_cb(const geometry_msgs::PoseStamped& msg) 
{
  double t = ros::Time::now().toSec() - start_time ;
  tf::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
  tf::Matrix3x3 R(q);
  double roll, pitch, yaw;
  R.getRPY(roll, pitch, yaw);
  ofstream ufile;
  ufile.open("/home/vulcan3/Documents/bags/copter_FLIGHT" + filename_ +".txt", std::ios::out | std::ios::app);
  ufile << t << ", " << msg.pose.position.x << ", " << msg.pose.position.y << ", " << msg.pose.position.z << ", " << yaw << "\n" ;
  ufile.close();	
}

void boat_cb(const geometry_msgs::PoseStamped& msg) 
{
  double t = ros::Time::now().toSec() - start_time ;
  tf::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
  tf::Matrix3x3 R(q);
  double roll, pitch, yaw;
  R.getRPY(roll, pitch, yaw);
  ofstream ufile;
  ufile.open("/home/vulcan3/Documents/bags/boat_" + filename_ +".txt", std::ios::out | std::ios::app);
  ufile << t << ", " << msg.pose.position.x << ", " << msg.pose.position.y << ", " << msg.pose.position.z << ", " << yaw << "\n" ;
  ufile.close();	
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "proj_log_node");
  ros::NodeHandle nh;
  start_time = ros::Time::now().toSec();
  filename_ = date_filename();

  ofstream ufile;
  ufile.open("/home/vulcan3/Documents/bags/copter_FLIGHT"+filename_ +".txt");
  ufile << "TIME, x, y, z, yaw \n";
  ufile.close();

  ufile.open("/home/vulcan3/Documents/bags/boat_"+filename_ +".txt");
  ufile << "TIME, x, y, z, yaw \n";
  ufile.close();

  ros::Subscriber copter_sub = nh.subscribe("/copter/mavros/local_position/pose", 1, copter_cb);
//  ros::Subscriber boat_sub = nh.subscribe("/boat/mavros/local_position/pose", 1, boat_cb);

  ros::spin();
  return 0;
}
