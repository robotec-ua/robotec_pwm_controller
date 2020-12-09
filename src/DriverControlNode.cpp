#include "driver_control/RobotHWImplementation.h"

int main(int argc, char** argv) {
	// Initialize the node
    ros::init(argc, argv, "single_joint_hardware_interface");
    ros::NodeHandle nh;

	// Multiple threads for controller service callback \
	 	and for the Service client callback used to get the \
	 	feedback from Ardiuno
    ros::MultiThreadedSpinner spinner(2);

	// Create new object to control the motors
	RobotHWImplementation ROBOT(nh);

	// ROS Loop
    spinner.spin();

    return 0;
}
