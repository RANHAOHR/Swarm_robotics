#include <multiAgent_system_modeling/multi_agent_controller.h>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "des_state_publisher");
    ros::NodeHandle nh;
 
    multiAgentController Controller(&nh);
    ros::Duration(1).sleep();
    Controller.basicController();
   while(ros::ok()){

//       Controller.avoidController();
       ros::spinOnce();
   }
    return 0;
}
