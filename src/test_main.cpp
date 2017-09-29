#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

//These names come from the way the joints are referred to in the simulator. They are substantially more compact, and no LESS obtuse.
geometry_msgs::Twist vel;

void callback_1(const nav_msgs::Odometry::ConstPtr& odom){  // the Odometry is global
//    ROS_INFO("Inside callback");
    geometry_msgs::PoseWithCovariance  positions = odom->pose;

    if(positions.pose.position.x >= 5){
        vel.linear.x = -0.5;
    }
    else if(positions.pose.position.x <= 0){
        vel.linear.x = 0.5;
    }

}

//
//void callback_2(const geometry_msgs::JointState::ConstPtr& odom){
//	std::vector<double> positions = odom->position;
//
//	std_msgs::Float64 m2_1_0;
//
//	m2_1_0.data = positions[0];
//
//	j2_1_0.publish(m2_1_0);
//
//}
//
//void callback_3(const geometry_msgs::JointState::ConstPtr& odom){
//	std::vector<double> positions = odom->position;
//
//	std_msgs::Float64 m2_1_0;
//
//	m2_1_0.data = positions[0];
//
//	j2_1_0.publish(m2_1_0);
//
//}

int main(int argc, char **argv) {
	ros::init(argc, argv, "swarm_test");
	ros::NodeHandle nh;

    ros::Publisher robot1 = nh.advertise<geometry_msgs::Twist>("/robot1/wheels_position_controller/cmd_vel", 1, true);
    ros::Publisher robot2 = nh.advertise<geometry_msgs::Twist>("/robot2/wheels_position_controller/cmd_vel", 1, true);
    ros::Publisher robot3 = nh.advertise<geometry_msgs::Twist>("/robot3/wheels_position_controller/cmd_vel", 1, true);


    vel.linear.x = 1;

	ros::Subscriber sub1 = nh.subscribe("/robot1/wheels_position_controller/odom", 10, callback_1);
//	ros::Subscriber sub2 = nh.subscribe("/robot2/wheels_position_controller/odom", 10, callback_2);
//	ros::Subscriber sub3 = nh.subscribe("/robot3/wheels_position_controller/odom", 10, callback_3);

    ros::Rate loop_rate(50);

	while(ros::ok()){

		ros::spinOnce();

//        ROS_INFO_STREAM("THE X VEL IS : " << vel.linear.x);
        robot1.publish(vel);
        robot2.publish(vel);
        robot3.publish(vel);

        loop_rate.sleep();
	}
	
	return 0;
}
