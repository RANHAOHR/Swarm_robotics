#ifndef MULTI_AGENT_CONTROLLER_H_
#define MULTI_AGENT_CONTROLLER_H_

#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <queue>
#include <time.h>
#include <cstdlib>

using namespace std;
const double dt = 0.02; //send desired-state messages at fixed rate, e.g. 0.02 sec = 50Hz
//dynamic parameters: should be tuned for target system

class multiAgentController {
private:
    ros::NodeHandle nh_;

    void sub1Callback(const nav_msgs::Odometry::ConstPtr& msg);
    void sub2Callback(const nav_msgs::Odometry::ConstPtr& msg);
    void sub3Callback(const nav_msgs::Odometry::ConstPtr& msg);

    void lidar1Callback(const sensor_msgs::LaserScan& msg);
    void lidar2Callback(const sensor_msgs::LaserScan& msg);
    void lidar3Callback(const sensor_msgs::LaserScan& msg);

    ros::Subscriber sub1;
    ros::Subscriber sub2;
    ros::Subscriber sub3;

    ros::Subscriber lidar1;
    ros::Subscriber lidar2;
    ros::Subscriber lidar3;

    ros::Publisher o1_dist;
    ros::Publisher o2_dist;
    ros::Publisher o3_dist;

    std_msgs::Float64 distance_1;
    std_msgs::Float64 distance_2;
    std_msgs::Float64 distance_3;

    ros::Publisher o1_angle;
    ros::Publisher o2_angle;
    ros::Publisher o3_angle;

    std_msgs::Float64 angle_1;
    std_msgs::Float64 angle_2;
    std_msgs::Float64 angle_3;

    ros::Publisher geo_twist1;
    ros::Publisher geo_twist2;
    ros::Publisher geo_twist3;
public: 

    double delta_t;

    double des_x1;
    double des_y1;

    double des_x2;
    double des_y2;

    double des_x3;
    double des_y3;

    double delta_speed1;
    double delta_speed2;
    double delta_speed3;
    double delta_w1;
    double delta_w2;
    double delta_w3;

    bool turn_r1;
    bool turn_r2;
    bool turn_r3;

    bool turn_w1;
    bool turn_w2;
    bool turn_w3;

    bool ORIENT;
    bool GO;

    bool AVOID;
    bool RESCHEDULE;
    int sche_cnt;
    int last_sche_cnt;
    bool avoid_go;

    double relative_theta1;
    double relative_theta2;
    double relative_theta3;

    std::vector<double> twist_1;
    std::vector<double> twist_2;
    std::vector<double> twist_3;
    double v1_max;
    double v2_max;
    double v3_max;

    double w1_max;
    double w2_max;
    double w3_max;

    double des_theta1;
    double des_theta2;
    double des_theta3;

    std::vector< std::vector<double> > obs_s1;
    std::vector< std::vector<double> > obs_s2;
    std::vector< std::vector<double> > obs_s3;

    bool avoidS1;
    bool avoidS2;
    bool avoidS3;

    geometry_msgs::Twist cmd1;
    geometry_msgs::Twist cmd2;
    geometry_msgs::Twist cmd3;

    geometry_msgs::Pose odom1;
    geometry_msgs::Pose odom2;
    geometry_msgs::Pose odom3;

    multiAgentController(ros::NodeHandle *nh);//constructor  

    void goToPosition();
    void reOrient();
    void avoidController();
    void reCompute();
    void turnAngle(double des_theta);
    void scheduler();
    std::vector<double> pointTopoint(geometry_msgs::Pose &odom, double des_x, double des_y, double &des_theta, double &relative_theta);

};

#endif
