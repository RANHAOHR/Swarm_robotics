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

    delat_t = 50; //the traveling time fpor robot to go to a point

    cmd1.linear.x = 0; //initial velocities
    cmd2.linear.x = 0;
    cmd3.linear.x = 0;

    des_x1 = 30; //set of destination positions
    des_y1 = 0.5;

    des_x2 = 30;
    des_y2 = -0.5;

    des_x3 = 29.5;
    des_y3 = 0;   

    ros::Duration(1).sleep(); //wait for callbacks on
    ros::spinOnce(); //to get the odom callback

    v1_max = pointTopoint(odom1, des_x1, des_y1); //get the 
    v2_max = pointTopoint(odom2, des_x2, des_y2);
    v3_max = pointTopoint(odom3, des_x3, des_y3);

    delta_speed1 = 2 * v1_max / delat_t; //spped steps
    delta_speed2 = 2 * v2_max / delat_t;
    delta_speed3 = 2 * v3_max / delat_t;
    turn_r1 = true; //velocity turn point
    turn_r2 = true;
    turn_r3 = true;

    /* the flags for sensing region*/
    avoidS1 = false;
    avoidS2 = false;
    avoidS3 = false;
    avoidS4 = false;
    avoidS5 = false;

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
    double angle_increment_ = msg.angle_increment;
   
    double s1_upper_index = (int) ((M_PI /6 - angle_min_)/angle_increment_);
    double s1_lower_index = (int) ((-M_PI /6 - angle_min_)/angle_increment_);
    // double front_ping_index_ = (int) ((0.0 -angle_min_)/angle_increment_);
    double s1_dist = 0.0;
    int cnt = 0;
    int void_cnt = 0;

    for (int i = s1_lower_index; i < s1_upper_index; ++i)
    {
        if (!isinf(msg.ranges[i]))
          {
            s1_dist += msg.ranges[i];
            cnt += 1;
          } else{
            void_cnt += 1; //no obstacles
          } 
    }
    if (abs(void_cnt - abs(s1_upper_index - s1_lower_index)) > 2 )
    {
       s1_dist = s1_dist / cnt;
       avoidS1 = true;  
    } 
    else avoidS1 = false;

    double s4_upper_index = (int) ((1.6 - angle_min_)/angle_increment_);
    double s4_lower_index = (int) ((1.4 - angle_min_)/angle_increment_);

    //region 4
    double s4_dist = 0.0;
    cnt = 0;
    void_cnt = 0;
//    ROS_INFO_STREAM("upper_index: "<< s4_upper_index);
//    ROS_INFO_STREAM("lower_index: "<< s4_lower_index);
    for (int i = s4_lower_index; i < s4_upper_index; ++i)
    {
        if (!isinf(msg.ranges[i]))
        {
            s4_dist += msg.ranges[i];
            cnt += 1;
        } else{
            void_cnt += 1; //no obstacles
        }
    }
    if (abs(void_cnt - abs(s4_upper_index - s4_lower_index)) > 2 )
    {
        s4_dist = s4_dist / cnt;
        avoidS4 = true;
    }
    else avoidS4 = false;

//    ROS_INFO_STREAM("-------------s1_dist R1: " << s1_dist);
//    ROS_INFO_STREAM("-------------s4_dist R1: " << s4_dist);
//    ROS_INFO_STREAM("avoidS1 : " << avoidS1);

}

void multiAgentController::lidar2Callback(const sensor_msgs::LaserScan& msg)
{
    double angle_min_ = msg.angle_min;
    double angle_max_ = msg.angle_max;
    double angle_increment_ = msg.angle_increment;

    double s2_upper_index = (int) ((M_PI /6 - angle_min_)/angle_increment_);
    double s2_lower_index = (int) ((-M_PI /6 - angle_min_)/angle_increment_);
    double s2_dist = 0.0;
    int cnt = 0;
    int void_cnt = 0;

    for (int i = s2_lower_index; i < s2_upper_index; ++i)
    {
        if (!isinf(msg.ranges[i]))
        {
            s2_dist += msg.ranges[i];
            cnt += 1;
        } else{
            void_cnt += 1; //no obstacles
        }
    }
    if (abs(void_cnt - abs(s2_upper_index - s2_lower_index)) > 2 )
    {
        s2_dist = s2_dist / cnt;
        avoidS2 = true;
    }
    else avoidS2 = false;

    double s3_upper_index = (int) ((-1.4 - angle_min_)/angle_increment_);
    double s3_lower_index = (int) ((-1.6 - angle_min_)/angle_increment_);
    //region 4
    double s3_dist = 0.0;
    cnt = 0;
    void_cnt = 0;
    for (int i = s3_lower_index; i < s3_upper_index; ++i)
    {
        if (!isinf(msg.ranges[i]))
        {
            s3_dist += msg.ranges[i];
            cnt += 1;
        } else{
            void_cnt += 1; //no obstacles
        }
    }
    if (abs(void_cnt - abs(s3_upper_index - s3_lower_index)) > 2 )
    {
        s3_dist = s3_dist / cnt;
        avoidS3 = true;
    }
    else avoidS3 = false;

//    ROS_INFO_STREAM("-------------s2_dist R2: " << s2_dist);
//    ROS_INFO_STREAM("-------------s3_dist R2: " << s3_dist);
//    ROS_INFO_STREAM("avoidS1 : " << avoidS2);
}

void multiAgentController::lidar3Callback(const sensor_msgs::LaserScan& msg)
{
    double angle_min_ = msg.angle_min;
    double angle_max_ = msg.angle_max;
    double angle_increment_ = msg.angle_increment;

    double s5_upper_index = (int) ((2.5 - angle_min_)/angle_increment_);
    double s5_lower_index = (int) ((-2.5 - angle_min_)/angle_increment_);
    double pos_index_ = (int) ((M_PI - angle_min_)/angle_increment_);
    double neg_index_ = (int) ((-M_PI - angle_min_)/angle_increment_);
    double s5_dist = 0.0;
    int cnt = 0;
    int void_cnt = 0;
    // ROS_INFO_STREAM("upper_index: "<< s5_upper_index);
    // ROS_INFO_STREAM("lower_index: "<< s5_lower_index);
    // ROS_INFO_STREAM("pos_index_: "<< pos_index_);
    // ROS_INFO_STREAM("neg_index_: "<< neg_index_);
    for (int i = s5_upper_index; i < pos_index_; ++i)
    {
        if (!isinf(msg.ranges[i]))
        {
            s5_dist += msg.ranges[i];
            cnt += 1;
        } else{
            void_cnt += 1; //no obstacles
        }
    }

    for (int i = neg_index_; i < s5_lower_index; ++i)
    {
        if (!isinf(msg.ranges[i]))
        {
            s5_dist += msg.ranges[i];
            cnt += 1;
        } else{
            void_cnt += 1; //no obstacles
        }
    }

    if (abs(void_cnt - abs(abs(s5_upper_index - pos_index_) + abs(s5_lower_index - neg_index_))) > 2 )
    {
        s5_dist = s5_dist / cnt;
        avoidS5 = true;
    }
    else avoidS5 = false;

    ROS_INFO_STREAM("-------------s5_dist R3: " << s5_dist);
    ROS_INFO_STREAM("avoidS5 : " << avoidS5);
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

void multiAgentController::avoidController(){

}