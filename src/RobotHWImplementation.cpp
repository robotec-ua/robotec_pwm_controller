#include <driver_control/RobotHWImplementation.h>

RobotHWImplementation::RobotHWImplementation(ros::NodeHandle& nh) : nh_(nh) {
	std::string left_wheel_setpoint_topic,			//
				right_wheel_setpoint_topic,			//
				left_wheel_state_topic,				//
				right_wheel_state_topic,			//
				left_wheel_data_topic,				//
				right_wheel_data_topic;				//

	// Update parameters from
	nh_.param<std::string>("left_wheel_setpoint_topic", left_wheel_setpoint_topic, "/left_wheel/setpoint");
	nh_.param<std::string>("right_wheel_setpoint_topic", right_wheel_setpoint_topic, "/right_wheel/setpoint");
	nh_.param<std::string>("left_wheel_state_topic", left_wheel_state_topic, "/left_wheel/state");
	nh_.param<std::string>("right_wheel_state_topic", right_wheel_state_topic, "/right_wheel/state");
	nh_.param<std::string>("left_wheel_data_topic", left_wheel_state_topic, "/left_wheel");
	nh_.param<std::string>("right_wheel_data_topic", right_wheel_state_topic, "/right_wheel");

	// Wheels data publishers
	left_wheel_setpoint_pub = nh_.advertise<std_msgs::Float64>(left_wheel_setpoint_topic, 10);
	right_wheel_setpoint_pub = nh_.advertise<std_msgs::Float64>(right_wheel_setpoint_topic, 10);
	left_wheel_state_pub = nh_.advertise<std_msgs::Float64>(left_wheel_state_topic, 10);
	right_wheel_state_pub = nh_.advertise<std_msgs::Float64>(right_wheel_state_topic, 10);

	// Wheels data subscribers
	left_wheel_data = nh_.subscribe(left_wheel_data_topic, 10, &RobotHWImplementation::leftWheelCallback, this);
	right_wheel_data = nh_.subscribe(right_wheel_data_topic, 10, &RobotHWImplementation::rightWheelCallback, this);

	// Create timer-controlled interrupt
	ros::Duration update_freq = ros::Duration(1.0/LOOP_RATE);
	timer_event = nh_.createTimer(update_freq, &RobotHWImplementation::update, this);

	// Initialize the object
	init();

	// Reset controller manager after initializing
	controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
}

RobotHWImplementation::~RobotHWImplementation() {}

void RobotHWImplementation::init() {
	hardware_interface::JointStateHandle jointStateHandle;		// State handler
	hardware_interface::JointHandle jointVelocityHandle;	    // Velocity handler
	hardware_interface::JointHandle jointEffortHandle;			// Effort handler

	// Create joints
	for (int index = 0; index != JOINTS_AMOUNT; ++index) {
		// Create new name for a joint
		names[index] = "wheel" + std::to_string(index + 1);;

		// Create and register state handlers
		jointStateHandle = hardware_interface::JointStateHandle(names[index],
			 							&joint_position_[index],
										&joint_velocity_[index],
										&joint_effort_[index]);
		joint_state_interface_.registerHandle(jointStateHandle);

		// Create velocity handlers
		jointVelocityHandle = hardware_interface::JointHandle(jointStateHandle, &joint_velocity_command_[index]);
		joint_velocity_interface.registerHandle(jointVelocityHandle);
	}

	// Register all joints interfaces
    registerInterface(&joint_state_interface_);
	registerInterface(&joint_velocity_interface);
}

void RobotHWImplementation::update(const ros::TimerEvent& e) {
	// Update elapsed time
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);

	// Read new data
    //read();

	// Update controller manager
    controller_manager_->update(ros::Time::now(), elapsed_time_);

	// Output data to control the motors
    write(elapsed_time_);
}

void RobotHWImplementation::read() {

}

void RobotHWImplementation::write(ros::Duration elapsed_time) {
	std_msgs::Float64 left_wheel_setpoint,		// Left wheel desired speed
						right_wheel_setpoint,	// Right wheel desired speed
						left_wheel_state,		// Left wheel current speed
						right_wheel_state;		// Right wheel current speed

	// Push desired velocities into the message
	left_wheel_setpoint.data = joint_velocity_command_[0];
	right_wheel_setpoint.data = joint_velocity_command_[1];

	// Push current velocities into the message
	left_wheel_state.data = joint_velocity_[0];
	right_wheel_state.data = joint_velocity_[1];

	// Print the data
	ROS_INFO("[RobotHWImplementation::read()] left com. vel : %.2f, right com. vel : %.2f",
				joint_velocity_command_[0], joint_velocity_command_[1]);

	ROS_INFO("[RobotHWImplementation::read()] left mot. vel.: %.2f, right mot. vel.: %.2f",
				joint_velocity_[0], joint_velocity_[1]);

	// Publish data
	left_wheel_setpoint_pub.publish(left_wheel_setpoint);
	right_wheel_setpoint_pub.publish(right_wheel_setpoint);
	left_wheel_state_pub.publish(left_wheel_state);
	right_wheel_state_pub.publish(right_wheel_state);
}

void RobotHWImplementation::leftWheelCallback(const std_msgs::Float64MultiArray& msg) {
	// Update left wheel position
	joint_position_[0] = angles::from_degrees(msg.data[0]);

	// Update left wheel speed
	joint_velocity_[0] = msg.data[1];
}

void RobotHWImplementation::rightWheelCallback(const std_msgs::Float64MultiArray& msg) {
	// Update right wheel position
	joint_position_[1] = angles::from_degrees(msg.data[0]);

	// Update right wheel speed
	joint_velocity_[1] = msg.data[1];
}
