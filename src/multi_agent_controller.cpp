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

    delta_t = 50; //the traveling time fpor robot to go to a point

    cmd1.linear.x = 0; //initial velocities
    cmd2.linear.x = 0;
    cmd3.linear.x = 0;

    des_x1 = 20; //set of destination positions
    des_y1 = 10.5;

    des_x2 = 20;
    des_y2 = 9.5;

    des_x3 = 19.5;
    des_y3 = 10;   

    ros::Duration(1).sleep(); //wait for callbacks on
    ros::spinOnce(); //to get the odom callback

    twist_1 = pointTopoint(odom1, des_x1, des_y1); //get the 
    twist_2 = pointTopoint(odom2, des_x2, des_y2);
    twist_3 = pointTopoint(odom3, des_x3, des_y3);

    v1_max = twist_1[0];
    v2_max = twist_2[0];
    v3_max = twist_3[0];

    w1_max = twist_1[1];
    w2_max = twist_2[1];
    w3_max = twist_3[1];

    delta_speed1 = 2 * v1_max / delta_t; //spped steps
    delta_speed2 = 2 * v2_max / delta_t;
    delta_speed3 = 2 * v3_max / delta_t;

    delta_w1 = 2 * w1_max / delta_t; //spped steps
    delta_w2 = 2 * w2_max / delta_t;
    delta_w3 = 2 * w3_max / delta_t;

    turn_r1 = true; //velocity turn point
    turn_r2 = true;
    turn_r3 = true;
    turn_w1 = true; //velocity turn point
    turn_w2 = true;
    turn_w3 = true;
    /* the flags for sensing region*/
    avoidS1_front = false;
    avoidS1_left = false;
    avoidS1_right = false;
    avoidS2_front = false;
    avoidS2_left = false;
    avoidS2_right = false;
<<<<<<< HEAD
=======

    group_left = false;
    group_right = false;
>>>>>>> 327315dfe93f6511684429086a81950598370d0e

    orient = true;

    real_theta = 2 * atan2( odom1.orientation.z,  odom1.orientation.w);
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
   
<<<<<<< HEAD
    double upper_index = (int) ((M_PI /6 - angle_min_)/angle_increment_);
    double lower_index = (int) ((-M_PI /6 - angle_min_)/angle_increment_);
=======
    double upper_index = (int) ((M_PI /5 - angle_min_)/angle_increment_);
    double lower_index = (int) ((-M_PI /5 - angle_min_)/angle_increment_);
>>>>>>> 327315dfe93f6511684429086a81950598370d0e
    // double front_ping_index_ = (int) ((0.0 -angle_min_)/angle_increment_);
    double s1_front_dist = 0.0;
    int cnt = 0;
    int void_cnt = 0;

    for (int i = lower_index; i < upper_index; ++i)
    {
        if (!isinf(msg.ranges[i]))
          {
            s1_front_dist += msg.ranges[i];
            cnt += 1;
          } else{
            void_cnt += 1; //no obstacles
          } 
    }
    if (abs(void_cnt - abs(upper_index - lower_index)) > 2 )
    {
<<<<<<< HEAD
       s1_dist = s1_dist / cnt;
=======
       s1_front_dist = s1_front_dist / cnt;
>>>>>>> 327315dfe93f6511684429086a81950598370d0e
       avoidS1_front = true;  
    } 
    else avoidS1_front = false;

<<<<<<< HEAD
    upper_index = (int) ((1.6 - angle_min_)/angle_increment_);
    lower_index = (int) ((1.4 - angle_min_)/angle_increment_);

    //region 4
    double s1_dist_l = 0.0;
    cnt = 0;
    void_cnt = 0;
//    ROS_INFO_STREAM("upper_index: "<< s4_upper_index);
//    ROS_INFO_STREAM("lower_index: "<< s4_lower_index);
=======
    upper_index = (int) ((2.0 - angle_min_)/angle_increment_);
    lower_index = (int) ((1.5 - angle_min_)/angle_increment_);

    double s1_left_dist = 0.0;
    cnt = 0;
    void_cnt = 0;
>>>>>>> 327315dfe93f6511684429086a81950598370d0e
    for (int i = lower_index; i < upper_index; ++i)
    {
        if (!isinf(msg.ranges[i]))
        {
<<<<<<< HEAD
            s1_dist_l += msg.ranges[i];
=======
            s1_left_dist += msg.ranges[i];
>>>>>>> 327315dfe93f6511684429086a81950598370d0e
            cnt += 1;
        } else{
            void_cnt += 1; //no obstacles
        }
    }
    if (abs(void_cnt - abs(upper_index - lower_index)) > 2 )
    {
<<<<<<< HEAD
        s1_dist_l = s1_dist_l / cnt;
        avoidS1_left = true;
    }
    else avoidS1_left = false;

    upper_index = (int) ((-1.6 - angle_min_)/angle_increment_);
    lower_index = (int) ((-1.4 - angle_min_)/angle_increment_);

    //region 4
    double s1_dist_r = 0.0;
    cnt = 0;
    void_cnt = 0;
//    ROS_INFO_STREAM("upper_index: "<< s4_upper_index);
//    ROS_INFO_STREAM("lower_index: "<< s4_lower_index);
=======
        s1_left_dist = s1_left_dist / cnt;
        avoidS1_left = true;
    }
    else avoidS1_left = false;

    upper_index = (int) ((-1.5 - angle_min_)/angle_increment_);
    lower_index = (int) ((-2.0 - angle_min_)/angle_increment_);

    double s1_right_dist = 0.0;
    cnt = 0;
    void_cnt = 0;
>>>>>>> 327315dfe93f6511684429086a81950598370d0e
    for (int i = lower_index; i < upper_index; ++i)
    {
        if (!isinf(msg.ranges[i]))
        {
<<<<<<< HEAD
            s1_dist_r += msg.ranges[i];
=======
            s1_right_dist += msg.ranges[i];
>>>>>>> 327315dfe93f6511684429086a81950598370d0e
            cnt += 1;
        } else{
            void_cnt += 1; //no obstacles
        }
    }
    if (abs(void_cnt - abs(upper_index - lower_index)) > 2 )
    {
<<<<<<< HEAD
        s1_dist_r = s1_dist_r / cnt;
        avoidS1_right = true;
    }
    else avoidS1_right = false;


    ROS_INFO_STREAM("-------------s1_dist R1: " << s1_dist);
    ROS_INFO_STREAM("-------------s1_dist_l R1: " << s1_dist_r);
    ROS_INFO_STREAM("avoidS1_right : " << avoidS1_right);
=======
        s1_right_dist = s1_right_dist / cnt;
        avoidS1_right = true;
    }
    else avoidS1_right = false;
   // ROS_INFO_STREAM("-------------s1_front_dist R1: " << s1_front_dist);
   // ROS_INFO_STREAM("-------------s1_right_dist R1: " << s1_right_dist);
   // ROS_INFO_STREAM("avoidS1_front : " << avoidS1_front);
   // ROS_INFO_STREAM("avoidS1_right : " << avoidS1_right);
>>>>>>> 327315dfe93f6511684429086a81950598370d0e

}

void multiAgentController::lidar2Callback(const sensor_msgs::LaserScan& msg)
{
    double angle_min_ = msg.angle_min;
    double angle_max_ = msg.angle_max;
    double angle_increment_ = msg.angle_increment;

    double upper_index = (int) ((M_PI /5 - angle_min_)/angle_increment_);
    double lower_index = (int) ((-M_PI /5 - angle_min_)/angle_increment_);
    double s2_front_dist = 0.0;
    int cnt = 0;
    int void_cnt = 0;

    for (int i = lower_index; i < upper_index; ++i)
    {
        if (!isinf(msg.ranges[i]))
        {
            s2_front_dist += msg.ranges[i];
            cnt += 1;
        } else{
            void_cnt += 1; //no obstacles
        }
    }
    if (abs(void_cnt - abs(upper_index - lower_index)) > 2 )
    {
        s2_front_dist = s2_front_dist / cnt;
        avoidS2_front = true;
    }
    else avoidS2_front = false;

    upper_index = (int) ((-1.5 - angle_min_)/angle_increment_);
    lower_index = (int) ((-2.0 - angle_min_)/angle_increment_);
    //region 4
    double s2_right_dist = 0.0;
    cnt = 0;
    void_cnt = 0;
    for (int i = lower_index; i < upper_index; ++i)
    {
        if (!isinf(msg.ranges[i]))
        {
            s2_right_dist += msg.ranges[i];
            cnt += 1;
        } else{
            void_cnt += 1; //no obstacles
        }
    }
<<<<<<< HEAD
    if (abs(void_cnt - abs(s3_upper_index - s3_lower_index)) > 2 )
    {
        s3_dist = s3_dist / cnt;
        avoidS3 = true;
    }
    else avoidS3 = false;

//    ROS_INFO_STREAM("-------------s2_dist R2: " << s2_dist);
//    ROS_INFO_STREAM("-------------s3_dist R2: " << s3_dist);
//    ROS_INFO_STREAM("avoidS1_front : " << avoidS2);
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
=======
    if (abs(void_cnt - abs(upper_index - lower_index)) > 2 )
>>>>>>> 327315dfe93f6511684429086a81950598370d0e
    {
        s2_right_dist = s2_right_dist / cnt;
        avoidS2_right = true;
    }
    else avoidS2_right = false;

    upper_index = (int) ((2.0 - angle_min_)/angle_increment_);
    lower_index = (int) ((1.5 - angle_min_)/angle_increment_);
    //region 4
    double s2_left_dist = 0.0;
    cnt = 0;
    void_cnt = 0;
    for (int i = lower_index; i < upper_index; ++i)
    {
        if (!isinf(msg.ranges[i]))
        {
            s2_left_dist += msg.ranges[i];
            cnt += 1;
        } else{
            void_cnt += 1; //no obstacles
        }
    }
    if (abs(void_cnt - abs(upper_index - lower_index)) > 2 )
    {
        s2_left_dist = s2_left_dist / cnt;
        avoidS2_left = true;
    }
    else avoidS2_left = false;
   // ROS_INFO_STREAM("-------------s2_dist R2: " << avoidS2_right);
   // ROS_INFO_STREAM("-------------s2_right_dist R2: " << s2_right_dist);
   // ROS_INFO_STREAM("avoidS2_right : " << avoidS2_right);

<<<<<<< HEAD
//    ROS_INFO_STREAM("-------------s5_dist R3: " << s5_dist);
//    ROS_INFO_STREAM("avoidS5 : " << avoidS5);
=======
>>>>>>> 327315dfe93f6511684429086a81950598370d0e
}

void multiAgentController::lidar3Callback(const sensor_msgs::LaserScan& msg)
{
    double angle_min_ = msg.angle_min;
    double angle_max_ = msg.angle_max;
    double angle_increment_ = msg.angle_increment;

    double s5_upper_index = (int) ((2.5 - angle_min_)/angle_increment_);
    double s5_lower_index = (int) ((-2.5 - angle_min_)/angle_increment_);

}

std::vector<double> multiAgentController::pointTopoint(geometry_msgs::Pose &odom, double des_x, double des_y)
{
    std::vector<double> twist_max;
    twist_max.resize(2);
    double cur_x = odom.position.x;
    double cur_y = odom.position.y;
    double delta_x = des_x - cur_x;
    double delta_y = des_y - cur_y;
    des_theta = atan2(delta_y, delta_x);
    double travel_dist = delta_x / cos(des_theta);
    ROS_INFO_STREAM("des_theta" << des_theta);
    ROS_INFO_STREAM("travel_dist" << travel_dist);
    
    double w1_max = 2 * des_theta / delta_t;
    double vx_max = 2 * travel_dist / delta_t;

    twist_max[0] = vx_max;
    twist_max[1] = w1_max;

    return twist_max;

}

void multiAgentController::reOrient(){

    while(orient){
        ros::spinOnce();
        ROS_INFO_STREAM("w1_max "<< w1_max);
        ROS_INFO_STREAM("cmd1.angular.z"<< cmd1.angular.z);
        if (cmd1.angular.z < w1_max && turn_w1) {cmd1.angular.z += delta_w1 * dt;}
        else if (cmd1.angular.z <= 0) {cmd1.angular.z = 0;} //we are not going back in this case
        else {
            cmd1.angular.z = cmd1.angular.z - delta_w1 * dt;
            turn_w1 = false;
        }
        real_theta = 2 * atan2( odom1.orientation.z,  odom1.orientation.w);
        ROS_INFO_STREAM(" test_angle_real "<< real_theta);

        if (cmd2.angular.z < w2_max && turn_w2) {cmd2.angular.z += delta_w2 * dt;}
        else if (cmd2.angular.z <= 0) {cmd2.angular.z = 0;} //we are not going back in this case
        else {
            cmd2.angular.z = cmd2.angular.z - delta_w2 * dt;
            turn_w2 = false;
        }

        if (cmd3.angular.z < w3_max && turn_w3) {cmd3.angular.z += delta_w3 * dt;}
        else if (cmd3.angular.z <= 0) {cmd3.angular.z = 0;} //we are not going back in this case
        else {
            cmd3.angular.z = cmd3.angular.z - delta_w3 * dt;
            turn_w3 = false;
        }
        geo_twist1.publish(cmd1);
        geo_twist2.publish(cmd2);
        geo_twist3.publish(cmd3);

        if (abs(real_theta - des_theta) < 0.02)
        {
            orient = false;
            break;
        }
        ros::Duration(dt).sleep();
    }


}
void multiAgentController::goToPosition(){
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
    ROS_INFO_STREAM("odom1.position.x"<< odom1.position.x << " odom y is: " << odom1.position.y);
    ROS_INFO_STREAM("odom2.position.x"<< odom2.position.x << " odom y is: " << odom2.position.y);
    ROS_INFO_STREAM("odom3.position.x"<< odom3.position.x << " odom y is: " << odom3.position.y);

    geo_twist1.publish(cmd1);
    geo_twist2.publish(cmd2);
    geo_twist3.publish(cmd3);

    ros::Duration(dt).sleep();

}

void multiAgentController::avoidController(){
    if (avoidS1_front)
    {
<<<<<<< HEAD
        ROS_INFO_STREAM("IN avoid S1");
        cmd1.angular.z = 0.1;
        cmd2.angular.z = 0.1;
        cmd3.angular.z = 0.1;
=======
        cmd1.linear.x = 0.0;
        cmd2.linear.x = 0.0;
        cmd3.linear.x = 0.0;

        cmd1.angular.z = 0.2;
        cmd2.angular.z = 0.2;
        cmd3.angular.z = 0.2;
>>>>>>> 327315dfe93f6511684429086a81950598370d0e

        geo_twist1.publish(cmd1);
        geo_twist2.publish(cmd2);
        geo_twist3.publish(cmd3);
<<<<<<< HEAD

    }

    if (!avoidS1_front && avoidS4){
        ROS_INFO_STREAM("Stop w");
        cmd1.angular.z = 0.0;
        cmd2.angular.z = 0.0;
        cmd3.angular.z = 0.0;
        cmd1.linear.x = 0.1;
        cmd2.linear.x = 0.1;
        cmd3.linear.x = 0.1;

        geo_twist1.publish(cmd1);
        geo_twist2.publish(cmd2);
        geo_twist3.publish(cmd3);

    }
    if(!avoidS1_front && !avoidS4) {
=======
    }

    if (avoidS1_right && !avoidS1_front)
    {
        cmd1.angular.z = 0.0;
        cmd2.angular.z = 0.0;
        cmd3.angular.z = 0.0;

        cmd1.linear.x = 0.1;
        cmd2.linear.x = 0.1;
        cmd3.linear.x = 0.1;
        group_left = true;

        geo_twist1.publish(cmd1);
        geo_twist2.publish(cmd2);
        geo_twist3.publish(cmd3);
    }


    if (!avoidS1_right && !avoidS1_front)
    {
        cmd1.angular.z = 0.0;
        cmd2.angular.z = 0.0;
        cmd3.angular.z = 0.0;

>>>>>>> 327315dfe93f6511684429086a81950598370d0e
        cmd1.linear.x = 0.0;
        cmd2.linear.x = 0.0;
        cmd3.linear.x = 0.0;

<<<<<<< HEAD
=======
        geo_twist1.publish(cmd1);
        geo_twist2.publish(cmd2);
        geo_twist3.publish(cmd3);
    }

    if (!avoidS1_front && avoidS2_right && group_left)
    {
>>>>>>> 327315dfe93f6511684429086a81950598370d0e
        cmd1.angular.z = 0.0;
        cmd2.angular.z = 0.0;
        cmd3.angular.z = 0.0;

<<<<<<< HEAD
        geo_twist1.publish(cmd1);
        geo_twist2.publish(cmd2);
        geo_twist3.publish(cmd3);

        /*recompute the point-to-point path */
    }
=======
        cmd1.linear.x = 0.1;
        cmd2.linear.x = 0.1;
        cmd3.linear.x = 0.1;

        geo_twist1.publish(cmd1);
        geo_twist2.publish(cmd2);
        geo_twist3.publish(cmd3);
    }

>>>>>>> 327315dfe93f6511684429086a81950598370d0e
    ros::Duration(dt).sleep();
}