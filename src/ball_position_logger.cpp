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

void pose_cb(const geometry_msgs::PoseStamped& msg) 
{
  double t = ros::Time::now().toSec() - start_time ;
  ofstream ufile;
  ufile.open("/home/vulcan3/Documents/ICUAS_LOGS/ball_pose_" + filename_ +".txt", std::ios::out | std::ios::app);
  ufile << t << ", " << msg.pose.position.x << ", " << msg.pose.position.y << ", " << msg.pose.position.z << "\n" ;
  ufile.close();
	
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "projball_log_node");
  ros::NodeHandle nh;
	start_time = ros::Time::now().toSec();
	filename_ = date_filename();

	ofstream ufile;
	ufile.open("/home/vulcan3/Documents/ICUAS_LOGS/ball_pose_"+filename_ +".txt");
	ufile << "TIME, x, y, z \n";
	ufile.close();


  ros::Subscriber pose_sub = nh.subscribe("/red/ball/pose", 1, pose_cb);

	ros::spin();
  return 0;
}
