#pragma once

#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <sensor_msgs/JointState.h>
#include <angles/angles.h>
#include <string>
#include <std_msgs/Float64.h>
#include <cstdint>

class RobotHWImplementation : public hardware_interface::RobotHW {
protected:
	static const uint8_t WHEELS_AMOUNT = 2;		// Amount of joints to create

	ros::Publisher left_wheel_setpoint_pub,		// Desirable value of speed to achieve on the left wheel
					right_wheel_setpoint_pub;	// Desirable value of speed to achieve on the right wheel

    hardware_interface::JointStateInterface _state_interface;			// Interface for checking state of joints (ros_control)
	hardware_interface::VelocityJointInterface _velocity_interface;		// Velocity controller interface (ros_control)

    double _positions[WHEELS_AMOUNT],						// Array of positions
	 		_velocities[WHEELS_AMOUNT],						// Array of velocities
			_efforts[WHEELS_AMOUNT],						// Array of applied efforts
			_effort_commands[WHEELS_AMOUNT],				// Array of efforts to handle joints control
			_velocity_commands[WHEELS_AMOUNT],				// Array of velocity to handle joints control
			_rate;

    ros::NodeHandle nh_;										// ROS node instance
    ros::Timer timer_event;										// ROS timer instance
    ros::Duration elapsed_time_;								// Elapsed time since last operation
    boost::shared_ptr<controller_manager::ControllerManager> 	// controller_manager instance 
		controller_manager_;

public:
	/**
	 * Constructor
	 * @param nh node handler object
	 */
	RobotHWImplementation(ros::NodeHandle& nh);

	/**
	 * Destructor
	 */
	~RobotHWImplementation();

	/**
	 * Initialization of the controllers for ControllerManager
	 */
	void controllerInitialization();

	/**
	 * Read, calculate and write data on a time event
	 * @param event current event 
	 */
	void update(const ros::TimerEvent& event);

	/**
	 * The method is dedicated to publishing the data when triggered by the timer event
	 * @param elapsed_time the time elapsed from the previous trigger time
	 */
	void write(ros::Duration elapsed_time);

	/**
	 * The callback for the left wheel velocity
	 * @param msg the received message
	 */
	void leftWheelVelocityCallback(const std_msgs::Float64&);

	/**
	 * The callback for the left wheel degrees of rotation
	 * @param msg the received message
	 */
	void leftWheelDegreesCallback(const std_msgs::Float64&);

	/**
	 * The callback for the right wheel velocity
	 * @param msg the received message
	 */
	void rightWheelVelocityCallback(const std_msgs::Float64&);

	/**
	 * The callback for the right wheel degrees of rotation
	 * @param msg the received message
	 */
	void rightWheelDegreesCallback(const std_msgs::Float64&);
};
