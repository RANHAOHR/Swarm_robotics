#include <multiAgent_system_modeling/multi_agent_controller.h>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "des_state_publisher");
    ros::NodeHandle nh;
 
    multiAgentController Controller(&nh);
    ros::Duration(1).sleep();

    while(ros::ok()){
<<<<<<< HEAD
        // Controller.goToPosiiton();
        Controller.avoidController();
=======
    	Controller.reOrient();
        Controller.goToPosition();
        // Controller.avoidController();
>>>>>>> 327315dfe93f6511684429086a81950598370d0e
        ros::spinOnce(); 
    }
    return 0;
}
