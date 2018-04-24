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

    delta_t = 50; //the traveling time for robot to go to a point

    des_x1 = 25; //set of destination positions
    des_y1 = 0.5;

    des_x2 = 25;
    des_y2 = -0.5;

    des_x3 = 24.5;
    des_y3 = 0;

    sche_cnt = 0;
    /* the flags for sensing region*/
    avoidS1 = false;
    avoidS2 = false;
    avoidS3 = false;

    RESCHEDULE = true;
    ORIENT = false;
    GO = false;
    AVOID = false;

    avoid_go = false;

    obs_s1.resize(0);
    obs_s2.resize(0);
    obs_s3.resize(0);

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

    double cur_orientation = 2 * atan2( odom1.orientation.z,  odom1.orientation.w);

    double upper_angle = angle_max_;
    double lower_angle = angle_min_;

    double upper_index = (int) ((upper_angle - angle_min_)/angle_increment_);
    double lower_index = (int) ((lower_angle- angle_min_)/angle_increment_); //avoid 2 and 3

    std::vector< std::vector<double> > obs_param;
    std::vector<double> temp_obs;
    temp_obs.resize(2);

    bool hit = false;

    double s1_dist = 0.0;
    int cnt = 0;
    double relative_angle_obs = 0;
    for (int i = lower_index; i < upper_index; ++i) {
        if (!isinf(msg.ranges[i]))
        {
            hit = true;
            s1_dist += msg.ranges[i];
            relative_angle_obs += (double) i * angle_increment_ + angle_min_;
            cnt += 1;
        } else{
            if(hit){
                s1_dist = s1_dist / cnt;
                relative_angle_obs = relative_angle_obs / cnt;
                temp_obs[0] = relative_angle_obs;
                temp_obs[1] = s1_dist;
                obs_param.push_back(temp_obs);
                s1_dist = 0;
                relative_angle_obs = 0;
                cnt = 0;
                hit = false;
            }

        }

    }

    if(hit){
        s1_dist = s1_dist / cnt;
        relative_angle_obs = relative_angle_obs / cnt;
        temp_obs[0] = relative_angle_obs;
        temp_obs[1] = s1_dist;
        obs_param.push_back(temp_obs);
    }

    /* sensing fusion and obstacles starts here */
    double teammate_2_angle = -1.57 - cur_orientation;
    double teammate_3_angle = -2.355 - cur_orientation;
    obs_s1.clear();
    for (int j = 0; j < obs_param.size(); ++j) {

        if( (obs_param[j][1] > 1.5 && obs_param[j][1] < 2.0 && fabs(obs_param[j][0] - teammate_2_angle) > 0.7 && fabs(obs_param[j][0] - teammate_3_angle) > 0.7) ){
            obs_s1.push_back(obs_param[j]);
        }

    }

//    ROS_INFO_STREAM("obs_param: " << obs_param.size());
    for (int l = 0; l < obs_param.size(); ++l) {
//        ROS_INFO_STREAM("obs_param: " << obs_param[l][0]);
//        ROS_INFO_STREAM("obs_param: " << obs_param[l][1]);
    }
//    ROS_INFO_STREAM("obs_s1.size() " << obs_s1.size());
    for (int k = 0; k < obs_s1.size(); ++k) {
//        ROS_INFO_STREAM("obs_s1: " << obs_s1[k][0]);
//        ROS_INFO_STREAM("obs_s1: " << obs_s1[k][1]);
        avoidS1 = true;

    }

    if( obs_s1.size() == 0){
        avoidS1 = false;
//        ROS_INFO_STREAM("NO OBSTACLES for robot 1!");
    }

    if(avoidS1 || avoidS2 || avoidS3){
        AVOID = true;
        RESCHEDULE = false;
        sche_cnt = 0;
    }else{
        sche_cnt += 1;
        AVOID = false;
    }
}

void multiAgentController::lidar2Callback(const sensor_msgs::LaserScan& msg)
{
    double angle_min_ = msg.angle_min;
    double angle_max_ = msg.angle_max;
    double angle_increment_ = msg.angle_increment;

    double cur_orientation = 2 * atan2( odom2.orientation.z,  odom2.orientation.w);

    double upper_angle = angle_max_;
    double lower_angle = angle_min_;

    double upper_index = (int) ((upper_angle - angle_min_)/angle_increment_);
    double lower_index = (int) ((lower_angle- angle_min_)/angle_increment_); //avoid 2 and 3

    std::vector< std::vector<double> > obs_param;
    std::vector<double> temp_obs;
    temp_obs.resize(2);

    bool hit = false;

    double s2_dist = 0.0;
    int cnt = 0;
    double relative_angle_obs = 0;
    for (int i = lower_index; i < upper_index; ++i) {
        if (!isinf(msg.ranges[i]))
        {
            hit = true;
            s2_dist += msg.ranges[i];
            relative_angle_obs += (double) i * angle_increment_ + angle_min_;
            cnt += 1;
        } else{
            if(hit){
                s2_dist = s2_dist / cnt;
                relative_angle_obs = relative_angle_obs / cnt;
                temp_obs[0] = relative_angle_obs;
                temp_obs[1] = s2_dist;
                obs_param.push_back(temp_obs);
                s2_dist = 0;
                relative_angle_obs = 0;
                cnt = 0;
                hit = false;
            }

        }

    }

    if(hit){
        s2_dist = s2_dist / cnt;
        relative_angle_obs = relative_angle_obs / cnt;
        temp_obs[0] = relative_angle_obs;
        temp_obs[1] = s2_dist;
        obs_param.push_back(temp_obs);
    }

    /* sensing fusion and obstacles starts here */
    double teammate_1_angle = 1.57 - cur_orientation;
    double teammate_3_angle = 2.355 - cur_orientation;
//    ROS_INFO_STREAM("obs_s2.size() " << obs_s2.size());
    obs_s2.clear();
    for (int j = 0; j < obs_param.size(); ++j) {

        if((obs_param[j][1] > 1.5 && obs_param[j][1] < 2.0 && fabs(obs_param[j][0] - teammate_1_angle) > 0.7 && fabs(obs_param[j][0] - teammate_3_angle) > 0.7 ) ){
            obs_s2.push_back(obs_param[j]);
        }

    }

    for (int k = 0; k < obs_s2.size(); ++k) {
//        ROS_INFO_STREAM("obs_s2: " << obs_s2[k][0]);
//        ROS_INFO_STREAM("obs_s2: " << obs_s2[k][1]);
        avoidS2 = true;

    }

    if( obs_s2.size() == 0){
        avoidS2 = false;
//        ROS_INFO_STREAM("NO OBSTACLES for robot 2!");
    }

}

void multiAgentController::lidar3Callback(const sensor_msgs::LaserScan& msg)
{

}

void multiAgentController::reCompute(){
    ros::Duration(1).sleep(); //wait for callbacks on
    ros::spinOnce(); //to get the odom callback

    twist_1 = pointTopoint(odom1, des_x1, des_y1, des_theta1, relative_theta1); //get destinations
    twist_2 = pointTopoint(odom2, des_x2, des_y2, des_theta2, relative_theta2);
    twist_3 = pointTopoint(odom3, des_x3, des_y3, des_theta3, relative_theta3);

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

    ORIENT = true;

}

std::vector<double> multiAgentController::pointTopoint(geometry_msgs::Pose &odom, double des_x, double des_y, double &des_theta, double &relative_theta)
{
    ros::spinOnce(); //callback rings
    std::vector<double> twist_max;
    twist_max.resize(2);
    double cur_x = odom.position.x;
    double cur_y = odom.position.y;
    double delta_x = des_x - cur_x;
    double delta_y = des_y - cur_y;
    des_theta = atan2(delta_y, delta_x);
    double travel_dist = delta_x / cos(des_theta);

    double curent_orientation = 2 * atan2( odom.orientation.z,  odom.orientation.w);
    ROS_INFO_STREAM(" ideal des_theta" << des_theta);
    ROS_INFO_STREAM("travel_dist" << travel_dist);
    ROS_INFO_STREAM("curent_orientation" << curent_orientation);

    relative_theta = des_theta - curent_orientation; //delta theta
    ROS_INFO_STREAM(" relative_theta" << relative_theta);
    double w_max = 2 * relative_theta / delta_t;
    double vx_max = 2 * travel_dist / delta_t;

    ROS_INFO_STREAM(" w_max" << w_max);
    twist_max[0] = vx_max;
    twist_max[1] = w_max;

    return twist_max;

}

void multiAgentController::scheduler(){

//    ROS_INFO_STREAM("RESCHEDULE " << RESCHEDULE);
//    ROS_INFO_STREAM("GO " << GO);
//    ROS_INFO_STREAM("AVOID " << AVOID);
//
//    ROS_INFO_STREAM("avoidS1  " << avoidS1);
//    ROS_INFO_STREAM("avoidS2 " << avoidS2);
//    ROS_INFO_STREAM("sche_cnt " << sche_cnt);

    if(sche_cnt == 1){
        RESCHEDULE = true;
        avoid_go = false;
    }
    if (RESCHEDULE){
        reCompute(); //get the position
        reOrient(); //turn a certain angle first
    }
    if(GO && !AVOID){
//        ROS_INFO_STREAM("IN GO");
        goToPosition();
        RESCHEDULE = false;
    }

    if(AVOID){
        GO = false; //make sure
        avoidController();
    }

}
void multiAgentController::reOrient(){
    cmd1.linear.x = 0;
    cmd2.linear.x = 0;
    cmd3.linear.x = 0;
    ROS_INFO_STREAM("IN reOrient");
    while(ORIENT){
        ros::spinOnce();
//        ROS_INFO_STREAM("w1_max "<< w1_max);
        if (fabs(cmd1.angular.z) < fabs(w1_max) && turn_w1) { cmd1.angular.z += delta_w1 * dt;}
        else {
            cmd1.angular.z -= delta_w1 * dt;
            turn_w1 = false;
        }
        double real_theta = 2 * atan2( odom1.orientation.z,  odom1.orientation.w);
//        ROS_INFO_STREAM(" test_angle_real "<< real_theta);
//        ROS_INFO_STREAM(" des_theta1 " << des_theta1);
        if (fabs(cmd2.angular.z) < fabs(w2_max) && turn_w2) {cmd2.angular.z += delta_w2 * dt; }
        else { 
            cmd2.angular.z -= delta_w2 * dt;
            turn_w2 = false;
        }

        if (fabs(cmd3.angular.z) < fabs(w3_max) && turn_w3) {cmd3.angular.z += delta_w3 * dt;}
        else {
            cmd3.angular.z -= delta_w3 * dt;
            turn_w3 = false;
        }

        geo_twist1.publish(cmd1);
        geo_twist2.publish(cmd2);
        geo_twist3.publish(cmd3);

        if (fabs(real_theta - des_theta1) < 0.005)
        {
            ROS_INFO_STREAM("---- Done orient ----");
            ORIENT = false;
            turn_w1 = true;
            turn_w2 = true;
            turn_w3 = true;
            GO = true;
            break;
        }
        ros::Duration(dt).sleep();
    }


}
void multiAgentController::goToPosition(){
    cmd1.angular.z = 0.0;
    cmd2.angular.z = 0.0;
    cmd3.angular.z = 0.0;

    ros::spinOnce();
//    ROS_INFO_STREAM("v1_max "<< v1_max);
//    ROS_INFO_STREAM("v2_max "<< v2_max);
//    ROS_INFO_STREAM("v3_max "<< v3_max);

//    ROS_INFO_STREAM("odom1.v.x "<< cmd1.linear.x);
//    ROS_INFO_STREAM("odom2.v.x "<< cmd2.linear.x);
//    ROS_INFO_STREAM("odom3.v.x "<< cmd3.linear.x);

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
    double test_orient = 2.0 * atan2(odom1.orientation.z , odom1.orientation.w);
    ROS_INFO_STREAM("orientation:" << test_orient);
    geo_twist1.publish(cmd1);
    geo_twist2.publish(cmd2);
    geo_twist3.publish(cmd3);

    if( fabs(odom1.position.x - des_x1) < 0.01){
        GO =false;
        ROS_INFO_STREAM("----DONE go to the current goal----");
    }
    ros::Duration(dt).sleep();


}


void multiAgentController::turnAngle(double des_theta){
    ROS_INFO_STREAM("IN TURN ANGLE");
    cmd1.linear.x = 0.0;
    cmd2.linear.x = 0.0;
    cmd3.linear.x = 0.0;

    double curent_orientation1 = 2 * atan2( odom1.orientation.z,  odom1.orientation.w);
//    ROS_INFO_STREAM("curent_orientation" << curent_orientation1);
    double relative1 = des_theta - curent_orientation1; //delta theta
    double w_max1 = 2 * relative1 / delta_t;
    double dw1 = 2 * w_max1 / delta_t; //spped steps

    double curent_orientation2 = 2 * atan2( odom2.orientation.z,  odom2.orientation.w);
//    ROS_INFO_STREAM("curent_orientation" << curent_orientation1);
    double relative_theta2 = des_theta - curent_orientation2; //delta theta
    double w_max2 = 2 * relative_theta2 / delta_t;
    double dw2 = 2 * w_max2 / delta_t; //spped steps

    double curent_orientation3 = 2 * atan2( odom3.orientation.z,  odom3.orientation.w);
//    ROS_INFO_STREAM("curent_orientation" << curent_orientation1);
    double relative_theta3 = des_theta - curent_orientation3; //delta theta
    double w_max3 = 2 * relative_theta3 / delta_t;
    double dw3 = 2 * w_max3 / delta_t; //spped steps

    bool turn_1 = true;
    bool turn_2 = true;
    bool turn_3 = true;
    bool orient = true;

    while(orient){
        ros::spinOnce();

        if (fabs(cmd1.angular.z) < fabs(w_max1) && turn_1) { cmd1.angular.z += dw1 * dt;}
        else {
            cmd1.angular.z -= dw1 * dt;
            turn_1 = false;
        }
//        ROS_INFO_STREAM(" cmd1.angular.z "<< cmd1.angular.z );
        double real_theta = 2 * atan2( odom1.orientation.z,  odom1.orientation.w);
//        ROS_INFO_STREAM(" test_angle_real "<< real_theta);
//        ROS_INFO_STREAM(" des_theta1 " << des_theta1);
        if (fabs(cmd2.angular.z) < fabs(w_max2) && turn_2) {cmd2.angular.z += dw2 * dt; }
        else {
            cmd2.angular.z -= dw2 * dt;
            turn_2 = false;
        }

        if (fabs(cmd3.angular.z) < fabs(w_max3) && turn_3) {cmd3.angular.z += dw3 * dt;}
        else {
            cmd3.angular.z -= dw3 * dt;
            turn_3 = false;
        }

        geo_twist1.publish(cmd1);
        geo_twist2.publish(cmd2);
        geo_twist3.publish(cmd3);

        if (fabs(real_theta - des_theta) < 0.005)
        {
            ROS_INFO_STREAM("---- Done orient ----");
            orient = false;
            avoid_go = true;
            break;
        }
        ros::Duration(dt).sleep();
    }

}

void multiAgentController::avoidController(){
    ros::spinOnce();
    avoid_go = false;
    ROS_INFO_STREAM("------IN OBSTACLES!------");
    if((avoidS1 || avoidS2) && !avoidS3){
        double obs_dist1 = 1000;
        double obs_angle1 = 1000;
        double obs_dist2 = 1000;
        double obs_angle2 = 1000;
        double main_angle;
        ROS_INFO_STREAM("obs_s1.size(): " << obs_s1.size());
        ROS_INFO_STREAM("obs_s2.size(): " << obs_s2.size());

        for (int k = 0; k < obs_s1.size(); ++k) {
                obs_angle1 = obs_s1[k][0];
                obs_dist1 = obs_s1[k][1];
//            ROS_INFO_STREAM("obs_angle1 : " << obs_s1[k][0]);
//            ROS_INFO_STREAM("obs_dist1: " << obs_s1[k][0]);
        }
        for (int k = 0; k < obs_s2.size(); ++k) {
                obs_angle2 = obs_s2[k][0];
                obs_dist2 = obs_s2[k][1];
//            ROS_INFO_STREAM("obs_angle2 : " << obs_s2[k][0]);
//            ROS_INFO_STREAM("obs_dist2: " << obs_s2[k][0]);
        }

        if( obs_dist1 <= obs_dist2 ){ main_angle = obs_angle1; }
        else{main_angle = obs_angle2;}

        double des_angle;
        ROS_INFO_STREAM("(main_angle) " << main_angle);
        if(fabs(main_angle) < 1.2){
            if(main_angle > 0){
                des_angle = main_angle - M_PI /1.8;
            }else{
                des_angle = M_PI /1.8 - main_angle;
            }
            turnAngle(des_angle);
        }else if(fabs(main_angle) > 1.2 ){
            avoid_go = true;
        }

        if(avoid_go){
            while(1){
                ros::spinOnce();
                ROS_INFO_STREAM("------- go avoiding--------");
                cmd1.linear.x = 0.2;
                cmd2.linear.x = 0.2;
                cmd3.linear.x = 0.2;

                cmd1.angular.z = 0.0;
                cmd2.angular.z = 0.0;
                cmd3.angular.z = 0.0;

                geo_twist1.publish(cmd1);
                geo_twist2.publish(cmd2);
                geo_twist3.publish(cmd3);

                ros::Duration(dt).sleep();
                if(!avoidS1 && !avoidS2){
                    break;
                }

            }

            for (int i = 0; i < 100; ++i) {
                ROS_INFO_STREAM("------- extra avoiding--------");
                cmd1.linear.x = 0.2;
                cmd2.linear.x = 0.2;
                cmd3.linear.x = 0.2;

                cmd1.angular.z = 0.0;
                cmd2.angular.z = 0.0;
                cmd3.angular.z = 0.0;

                geo_twist1.publish(cmd1);
                geo_twist2.publish(cmd2);
                geo_twist3.publish(cmd3);

                ros::Duration(dt).sleep();
            }
            sche_cnt = 0;
            RESCHEDULE = true;

        }

    }

}