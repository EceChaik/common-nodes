#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <fstream>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/JointState.h>
#include <time.h>
#include <cmath>
#include <math.h>
#include <cstdlib>
#include <ros/duration.h>
#include <chrono>

using namespace std;
string filename_;
string folder_name_;
double start_time;
#define PI 3.1416
#define Q10 3000
#define Q20 2065
#define q1_in 1.367
#define q2_in 0.209

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

double int2rad(double position, int id)
{
  int q0;
  if(id==1)
    q0 = Q10;
  else
    q0 = Q20;

  double deg = (position-q0)*(360/4096.0);	  
  double angle = deg*3.1416/180;
  return angle;
}

void uav_cb(const geometry_msgs::PoseStamped& msg)
{
  double t = ros::Time::now().toSec() - start_time ;
  tf::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
  tf::Matrix3x3 R(q);
  double roll, pitch, yaw;
  R.getRPY(roll, pitch, yaw);
  ofstream ufile;
  ufile.open("/home/vulcan3/Documents/force_LOGS/"+folder_name_+"/uav.txt", std::ios::out | std::ios::app);
  ufile << t << ", " << msg.pose.position.x << ", " << msg.pose.position.y << ", " << msg.pose.position.z << ", " << roll << ", " << pitch << ", " << yaw << "\n" ;
  ufile.close();
}

void rob_cb(const geometry_msgs::Vector3& msg){
  double t = ros::Time::now().toSec() - start_time ;
  double q1 = int2rad(msg.x,1) + q1_in;
  double q2 = int2rad(msg.y,2) + q2_in;
  ofstream ufile;
  ufile.open("/home/vulcan3/Documents/force_LOGS/"+folder_name_+"/arm.txt", std::ios::out | std::ios::app);
  ufile << t << ", " << q1 << ", " << q2 << "\n" ;
  ufile.close();	
}

void ft_cb(const geometry_msgs::WrenchStamped& msg){
  double t = ros::Time::now().toSec() - start_time ;
  ofstream ufile;
  ufile.open("/home/vulcan3/Documents/force_LOGS/"+folder_name_+"/force.txt", std::ios::out | std::ios::app);
  ufile << t << ", " << msg.wrench.force.x << ", " << msg.wrench.force.y << ", " << msg.wrench.force.z << ", " << msg.wrench.torque.x << ", " << msg.wrench.torque.y << ", " << msg.wrench.torque.z << "\n" ;
  ufile.close();	  
}

void mavros_cb(const mavros_msgs::AttitudeTarget& msg){
  double t = ros::Time::now().toSec() - start_time ;
  tf::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
  tf::Matrix3x3 R(q);
  double roll, pitch, yaw;
  R.getRPY(roll, pitch, yaw);
  ofstream ufile;
  ufile.open("/home/vulcan3/Documents/force_LOGS/"+folder_name_+"/control.txt", std::ios::out | std::ios::app);
  ufile << t << ", " << msg.thrust << ", " << roll << ", " << pitch << ", " << msg.body_rate.z << "\n" ;
  ufile.close();	  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "proj_log_node");
  ros::NodeHandle nh;
  start_time = ros::Time::now().toSec();
  filename_ = date_filename();
  folder_name_ = "test_"+filename_;
  string command = "mkdir -p /home/vulcan3/Documents/force_LOGS/"+folder_name_ ;
  const int dir_err = system(command.c_str());
  if (-1 == dir_err)
  {
	printf("Error creating directory!n");
	exit(1);
  }
  
  ofstream ufile;
  ufile.open("/home/vulcan3/Documents/force_LOGS/"+folder_name_+"/uav.txt");
  ufile << "TIME, x, y, z, roll, pitch, yaw \n";
  ufile.close();

  ufile.open("/home/vulcan3/Documents/force_LOGS/"+folder_name_+"/arm.txt");
  ufile << "TIME, q1, q2 \n";
  ufile.close();

  ufile.open("/home/vulcan3/Documents/force_LOGS/"+folder_name_+"/control.txt");
  ufile << "TIME, T, roll, pitch, yaw_r \n";
  ufile.close();

  ufile.open("/home/vulcan3/Documents/force_LOGS/"+folder_name_+"/force.txt");
  ufile << "TIME, fx, fy, fz, nx, ny, nz \n";
  ufile.close();

  
  ros::Subscriber uav_sub = nh.subscribe("/m100/odometry", 1, uav_cb);
  ros::Subscriber control_sub = nh.subscribe("/m100/mavros/setpoint_raw/attitude", 1, mavros_cb);
  ros::Subscriber robot = nh.subscribe("/m100/robot_state", 1, rob_cb);
  ros::Subscriber ft_sub = nh.subscribe("/m100/ft_sensor_topic", 1, ft_cb);

  ros::spin();
  return 0;
}
