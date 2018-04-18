#include <multiAgent_system_modeling/multi_agent_controller.h>

multiAgentController::multiAgentController(ros::NodeHandle *nh) : nh_( *nh ) {
	geo_twist1 = nh_.advertise <geometry_msgs::Twist> ("/robot1/wheels_position_controller/cmd_vel", 1, true);
	geo_twist2 = nh_.advertise <geometry_msgs::Twist> ("/robot2/wheels_position_controller/cmd_vel", 1, true);
	geo_twist3 = nh_.advertise <geometry_msgs::Twist> ("/robot3/wheels_position_controller/cmd_vel", 1, true);

	sub1 = nh_.subscribe("/robot1/wheels_position_controller/odom", 1, &multiAgentController::sub1Callback, this);
	sub2 = nh_.subscribe("/robot2/wheels_position_controller/odom", 1, &multiAgentController::sub2Callback, this);
	sub3 = nh_.subscribe("/robot3/wheels_position_controller/odom", 1, &multiAgentController::sub3Callback, this);

	lidar1 = nh_.subscribe("/robot1/laser_scan", 1, &multiAgentController::lidar1Callback, this);
	lidar2 = nh_.subscribe("/robot2/laser_scan", 1, &multiAgentController::lidar2Callback, this);
	lidar3 = nh_.subscribe("/robot3/laser_scan", 1, &multiAgentController::lidar3Callback, this);

    delat_t = 50;

    cmd1.linear.x = 0;
    cmd2.linear.x = 0;
    cmd3.linear.x = 0;

    des_x1 = 30;
    des_y1 = 0.5;

    des_x2 = 30;
    des_y2 = -0.5;

    des_x3 = 29.5;
    des_y3 = 0;   

    ros::Duration(1).sleep();
    ros::spinOnce();

    v1_max = pointTopoint(odom1, des_x1, des_y1);
    v2_max = pointTopoint(odom2, des_x2, des_y2);
    v3_max = pointTopoint(odom3, des_x3, des_y3);

    delta_speed1 = 2 * v1_max / delat_t;
    delta_speed2 = 2 * v2_max / delat_t;
    delta_speed3 = 2 * v3_max / delat_t;
    turn_r1 = true;
    turn_r2 = true;
    turn_r3 = true;
}

void multiAgentController::sub1Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom1 = msg->pose.pose;
}

void multiAgentController::sub2Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom2 = msg->pose.pose;
}

void multiAgentController::sub3Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom3 = msg->pose.pose;
}

void multiAgentController::lidar1Callback(const sensor_msgs::LaserScan& msg)
{
    double angle_min_ = msg.angle_min;
    double angle_max_ = msg.angle_max;
    double angle_increment_ = msg.angle_increment;
    double range_min_ = msg.range_min;
    double range_max_ = msg.range_max;

    double front_ping_index_ = (int) ((0.0 -angle_min_)/angle_increment_);
    double ping_dist_in_front_ = msg.ranges[front_ping_index_];

    ROS_INFO_STREAM("-------------test R1: " << ping_dist_in_front_);

}

void multiAgentController::lidar2Callback(const sensor_msgs::LaserScan& msg)
{
    double angle_min_ = msg.angle_min;
    double angle_max_ = msg.angle_max;
    double angle_increment_ = msg.angle_increment;
    double range_min_ = msg.range_min;
    double range_max_ = msg.range_max;

}

void multiAgentController::lidar3Callback(const sensor_msgs::LaserScan& msg)
{
    double angle_min_ = msg.angle_min;
    double angle_max_ = msg.angle_max;
    double angle_increment_ = msg.angle_increment;
    double range_min_ = msg.range_min;
    double range_max_ = msg.range_max;

    ROS_INFO_STREAM("angle_increment_ R3: " << angle_increment_);
}

double multiAgentController::pointTopoint(geometry_msgs::Pose &odom, double des_x, double des_y)
{
	ROS_INFO_STREAM("odom is "<< odom.position.x);

    double cur_x = odom.position.x;
    double cur_y = odom.position.y;
    double delta_x = des_x - cur_x;
    double delta_y = des_y - cur_y;

    double vx_max = 2 * delta_x / delat_t;

    return vx_max;

}

void multiAgentController::goToPosiiton(){
    ROS_INFO_STREAM("v1_max "<< v1_max);
    ROS_INFO_STREAM("v2_max "<< v2_max);
    ROS_INFO_STREAM("v3_max "<< v3_max);
    
    ROS_INFO_STREAM("odom1.v.x "<< cmd1.linear.x);
    ROS_INFO_STREAM("odom2.v.x "<< cmd2.linear.x);
    ROS_INFO_STREAM("odom3.v.x "<< cmd3.linear.x);
    if (cmd1.linear.x < v1_max && turn_r1) {cmd1.linear.x += delta_speed1 * dt;}
    else if (cmd1.linear.x <= 0) {cmd1.linear.x = 0;} //we are not going back in this case
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

    ros::Duration(dt).sleep();

}

