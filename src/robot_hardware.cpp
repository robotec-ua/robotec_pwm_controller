#include "robot_hardware.hpp"

RobotHWImplementation::RobotHWImplementation(ros::NodeHandle& nh) : nh_(nh) {
	ros::NodeHandle priv_nh("~");					// Private ROS node handler

	ros::Subscriber left_wheel_velocity,			// Left wheel velocity data subscriber
				right_wheel_velocity,				// Right wheel velocity data subscriber
				left_wheel_degrees,					// Left wheel degrees data subscriber
				right_wheel_degrees;				// Right wheel degrees data subscriber

	std::string left_wheel_setpoint_topic,			// Setpoint for the left wheel topic name
				right_wheel_setpoint_topic,			// Setpoint for the right wheel topic name
				left_wheel_velocity_topic,			// Velocity for the left wheel topic name
				right_wheel_velocity_topic,			// Velocity for the right wheel topic name
				left_wheel_degrees_topic,			// Degrees for the left wheel topic name
				right_wheel_degrees_topic;			// Degrees for the right wheel topic name

	// Update parameters from the launchfile
	priv_nh.param<std::string>("left_velocity", left_wheel_velocity_topic, "/left_wheel/velocity");
	priv_nh.param<std::string>("right_velocity", right_wheel_velocity_topic, "/right_wheel/velocity");
	priv_nh.param<std::string>("left_degrees", left_wheel_degrees_topic, "/left_wheel/degrees");
	priv_nh.param<std::string>("right_degrees", right_wheel_degrees_topic, "/right_wheel/degrees");
	priv_nh.param<std::string>("left_setpoint", left_wheel_setpoint_topic, "/left_wheel/setpoint");
	priv_nh.param<std::string>("right_setpoint", right_wheel_setpoint_topic, "/right_wheel/setpoint");
	priv_nh.param<double>("rate", _rate, 50);

	// Create data publishers
	left_wheel_setpoint_pub = nh_.advertise<std_msgs::Float64>(left_wheel_setpoint_topic, 1);
	right_wheel_setpoint_pub = nh_.advertise<std_msgs::Float64>(right_wheel_setpoint_topic, 1);

	// Create subscribers
	left_wheel_velocity = nh_.subscribe(left_wheel_velocity_topic, 1, &RobotHWImplementation::leftWheelVelocityCallback, this);
	right_wheel_velocity = nh_.subscribe(right_wheel_velocity_topic, 1, &RobotHWImplementation::rightWheelVelocityCallback, this);
	left_wheel_degrees = nh_.subscribe(left_wheel_degrees_topic, 1, &RobotHWImplementation::leftWheelDegreesCallback, this);
	right_wheel_degrees = nh_.subscribe(right_wheel_degrees_topic, 1, &RobotHWImplementation::rightWheelDegreesCallback, this);

	// Create timer-controlled interrupts
	ros::Duration update_freq = ros::Duration(1.0/_rate);
	timer_event = nh_.createTimer(update_freq, &RobotHWImplementation::update, this);

	// Initialize the controllers
	controllerInitialization();

	// Reset controller manager after initializing
	controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
}

RobotHWImplementation::~RobotHWImplementation() {}

void RobotHWImplementation::controllerInitialization() {
	hardware_interface::JointStateHandle state_handler;		// State handler
	hardware_interface::JointHandle joint_handler;	    	// Velocity handler
	std::string joint;										// Current joint name

	// Create joints
	for (int index = 0; index != WHEELS_AMOUNT; ++index) {
		// Create a new name for a joint
		joint = "wheel" + std::to_string(index + 1);

		// Create and register state handlers
		state_handler = hardware_interface::JointStateHandle(joint,
			 							&_positions[index],
										&_velocities[index],
										&_efforts[index]);
		_state_interface.registerHandle(state_handler);

		// Create velocity handlers
		joint_handler = hardware_interface::JointHandle(state_handler, &_velocity_commands[index]);
		_velocity_interface.registerHandle(joint_handler);
	}

	// Register all joints interfaces
    registerInterface(&_state_interface);
	registerInterface(&_velocity_interface);
}

void RobotHWImplementation::update(const ros::TimerEvent& event) {
	// Update elapsed time
    elapsed_time_ = ros::Duration(event.current_real - event.last_real);

	// Update controller manager
    controller_manager_->update(ros::Time::now(), elapsed_time_);

	// Output data to control the motors
    write(elapsed_time_);
}

void RobotHWImplementation::write(ros::Duration elapsed_time) {
	std_msgs::Float64 left_wheel_setpoint,		// Left wheel desired speed
						right_wheel_setpoint;	// Right wheel desired speed

	// Push desired velocities into the message
	left_wheel_setpoint.data = _velocity_commands[0];
	right_wheel_setpoint.data = _velocity_commands[1];

	// Publish setpoints
	left_wheel_setpoint_pub.publish(left_wheel_setpoint);
	right_wheel_setpoint_pub.publish(right_wheel_setpoint);
}

void RobotHWImplementation::leftWheelVelocityCallback(const std_msgs::Float64& msg) {
	// Update left wheel speed
	_velocities[0] = msg.data;
}

void RobotHWImplementation::leftWheelDegreesCallback(const std_msgs::Float64& msg) {
	// Update left wheel position
	_positions[0] = angles::from_degrees(msg.data);
}

void RobotHWImplementation::rightWheelVelocityCallback(const std_msgs::Float64& msg) {
	// Update right wheel speed
	_velocities[1] = msg.data;
}

void RobotHWImplementation::rightWheelDegreesCallback(const std_msgs::Float64& msg) {
	// Update right wheel position
	_positions[1] = angles::from_degrees(msg.data);
}
