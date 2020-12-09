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
#include <std_msgs/Float64MultiArray.h>

#define LOOP_RATE 100						// Update data 100 times per second
#define JOINTS_AMOUNT 2						// Amount of joints to create

class RobotHWImplementation : public hardware_interface::RobotHW {
protected:
	ros::Publisher left_wheel_setpoint_pub,		// Desirable value of speed to achieve on the left wheel
					right_wheel_setpoint_pub,	// Desirable value of speed to achieve on the right wheel
					left_wheel_state_pub,		// Current value of speed to achieve on the left wheel
					right_wheel_state_pub;		// Current value of speed to achieve on the right wheel

	ros::Subscriber left_wheel_data,
					right_wheel_data;

    hardware_interface::JointStateInterface joint_state_interface_;				// Interface for checking state of joints
	hardware_interface::VelocityJointInterface joint_velocity_interface;		// Velocity controller

	std::string names[JOINTS_AMOUNT];							// Array of names

    double joint_position_[JOINTS_AMOUNT],						// Array of positions
	 		joint_velocity_[JOINTS_AMOUNT],						// Array of velocities
			joint_effort_[JOINTS_AMOUNT],						// Array of applied efforts
			joint_effort_command_[JOINTS_AMOUNT],				// Array of efforts to handle joints control
			joint_velocity_command_[JOINTS_AMOUNT];				// Array of velocity to handle joints control

    ros::NodeHandle nh_;										// Main node
    ros::Timer timer_event;										// Timer
    ros::Duration elapsed_time_;								// Elapsed time since last operation
    boost::shared_ptr<controller_manager::ControllerManager> 	//
		controller_manager_;

public:
	RobotHWImplementation(ros::NodeHandle& nh);			// Constructor
	~RobotHWImplementation();							// Destructor
	void init();										// Initialize object
	void update(const ros::TimerEvent& e);				// Timer-controlled update for data
	void read();										// Read state of joints
	void write(ros::Duration elapsed_time);				// Apply efforts to joints
	void leftWheelCallback(const std_msgs::Float64MultiArray&);	//
	void rightWheelCallback(const std_msgs::Float64MultiArray&);	//
};
