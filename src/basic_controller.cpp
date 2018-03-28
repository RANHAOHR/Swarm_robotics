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
geometry_msgs::Pose odom1;
geometry_msgs::Pose odom2;
geometry_msgs::Pose odom3;

void sub1Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom1 = msg->pose.pose;
}

void sub2Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom2 = msg->pose.pose;
}

void sub3Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom3 = msg->pose.pose;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "des_state_publisher");
    ros::NodeHandle nh;
    geometry_msgs::Twist cmd1;
    geometry_msgs::Twist cmd2;
    geometry_msgs::Twist cmd3;

    double dt = 0.01;
    ros::Publisher geo_twist1 = nh.advertise <geometry_msgs::Twist> ("/robot1/wheels_position_controller/cmd_vel", 1, true);
    ros::Publisher geo_twist2 = nh.advertise <geometry_msgs::Twist> ("/robot2/wheels_position_controller/cmd_vel", 1, true);
    ros::Publisher geo_twist3 = nh.advertise <geometry_msgs::Twist> ("/robot3/wheels_position_controller/cmd_vel", 1, true);
    ros::Rate looprate(1 / dt); //timer for fixed publication rate
    //put some points in the path queue--hard coded here

    ros::Subscriber sub1 = nh.subscribe("/robot1/wheels_position_controller/odom", 1000, sub1Callback);
    ros::Subscriber sub2 = nh.subscribe("/robot2/wheels_position_controller/odom", 1000, sub2Callback);
    ros::Subscriber sub3 = nh.subscribe("/robot3/wheels_position_controller/odom", 1000, sub3Callback);
    cmd1.linear.x = 1;
    cmd2.linear.x = 1;
    cmd3.linear.x = 1;
    while(ros::ok()){
        ROS_INFO_STREAM("odom1.position.x"<< odom1.position.x);
        ROS_INFO_STREAM("odom2.position.x"<< odom2.position.x);
        ROS_INFO_STREAM("odom3.position.x"<< odom3.position.x);

        if(odom1.position.x>=10)
        {
            cmd1.linear.x = -1;
        }
        else if(odom1.position.x<=0){
            cmd1.linear.x = 1;
        }

        if(odom2.position.x>=10)
        {
            cmd2.linear.x = -1;

        }
        else if(odom2.position.x<=0){
            cmd2.linear.x = 1;

        }

        if(odom3.position.x>=10)
        {
            cmd3.linear.x = -1;
        }
        else if(odom3.position.x<=0){
            cmd3.linear.x = 1;
        }

        geo_twist1.publish(cmd1);
        geo_twist2.publish(cmd2);
        geo_twist3.publish(cmd3);

        ros::spinOnce();
        looprate.sleep();
    }

    return 0;

}
