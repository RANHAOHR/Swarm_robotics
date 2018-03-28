
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float64.h>

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <queue>
#include <time.h>
#include <cstdlib>

using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "des_state_publisher");
  ros::NodeHandle nh;
  //instantiate a desired-state publisher object
  SwarmControlAlgorithm swarm_control_algorithm(nh);
  
  ros::Publisher geo_twist = nh.advertise <geometry_msgs::Twist> ("robot1/cmd_vel", 1, true);

  ros::Rate looprate(1 / dt); //timer for fixed publication rate
  //put some points in the path queue--hard coded here

  ////swarm_1
    ros::spinOnce();
    //sleep for defined sample period, then do loop again

}
