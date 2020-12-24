#include "robot_hardware.hpp"

int main(int argc, char** argv) {
	// Initialize the node
    ros::init(argc, argv, "velocity_controller");
    ros::NodeHandle nh;
	ros::NodeHandle priv_nh("~");

	/* 	Multiple threads for controller service callback
		and for the Service client callback used to get the
	 	feedback from Ardiuno
	*/
    ros::MultiThreadedSpinner spinner(2);

	// Create new object to control the motors
	RobotHWImplementation hardware_implementation(nh);

	// ROS Loop
    spinner.spin();

    return 0;
}
