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

double delat_t = 50;

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

double pointTopoint(geometry_msgs::Pose &odom, double des_x, double des_y)
{
    double cur_x = odom.position.x;
    double cur_y = odom.position.y;
    double delta_x = des_x - cur_x;
    double delta_y = des_y - cur_y;

    double vx_max = 2 * delta_x / delat_t;

    return vx_max;

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
    cmd1.linear.x = 0;
    cmd2.linear.x = 0;
    cmd3.linear.x = 0;

    double des_x1 = 30;
    double des_y1 = 0.5;

    double des_x2 = 30;
    double des_y2 = -0.5;

    double des_x3 = 29.5;
    double des_y3 = 0;   

    double v1_max = pointTopoint(odom1, des_x1, des_y1);
    double v2_max = pointTopoint(odom2, des_x2, des_y2);
    double v3_max = pointTopoint(odom3, des_x3, des_y3);

    double pk = 2;

    double delta_speed1 = 2 * v1_max / delat_t;
    double delta_speed2 = 2 * v2_max / delat_t;
    double delta_speed3 = 2 * v3_max / delat_t;
    bool turn_r1 = true;
    bool turn_r2 = true;
    bool turn_r3 = true;

    while(ros::ok()){
        ROS_INFO_STREAM("v1_max "<< v1_max);
        ROS_INFO_STREAM("v2_max "<< v2_max);
        ROS_INFO_STREAM("v3_max "<< v3_max);
        
        ROS_INFO_STREAM("odom1.v.x "<< cmd1.linear.x);
        ROS_INFO_STREAM("odom2.v.x "<< cmd2.linear.x);
        ROS_INFO_STREAM("odom3.v.x "<< cmd3.linear.x);

        if (cmd1.linear.x < v1_max && turn_r1) {cmd1.linear.x += delta_speed1 * dt;}
        else if (cmd1.linear.x <= 0) {cmd1.linear.x = 0;}
        else {
            cmd1.linear.x = cmd1.linear.x - delta_speed1 * dt;
            turn_r1 = false;
        }

        if (cmd2.linear.x < v2_max && turn_r2) {cmd2.linear.x += delta_speed2 * dt;}
        else if (cmd2.linear.x <= 0) {cmd2.linear.x = 0;}
        else {
            cmd2.linear.x = cmd2.linear.x - delta_speed2 * dt;
            turn_r2 = false;
        }

        if (cmd3.linear.x < v3_max && turn_r3) {cmd3.linear.x += delta_speed3 * dt;}
        else if (cmd3.linear.x <= 0) {cmd3.linear.x = 0;}
        else {
            cmd3.linear.x = cmd3.linear.x - delta_speed3 * dt;
            turn_r3 = false;
        }
        ROS_INFO_STREAM("odom1.position.x"<< odom1.position.x);
        ROS_INFO_STREAM("odom2.position.x"<< odom2.position.x);
        ROS_INFO_STREAM("odom3.position.x"<< odom3.position.x);

        geo_twist1.publish(cmd1);
        geo_twist2.publish(cmd2);
        geo_twist3.publish(cmd3);

        looprate.sleep();

        ros::spinOnce();
        
    }

    return 0;

}
