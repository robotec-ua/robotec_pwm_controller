# Velocity Controller by Robotec
The package is an interlayer between the robot's navigation stack and actual hardware controllers. It computes separate velocities for wheels of a two-wheeled robot using command velocity and the known state of every wheel. 

## Table of contents
* [Project structure](#project_structure)
    * [Folders](#folders)
    * [Files](#files)
* [Class Hierarchy](#class_hierarchy)
    * [Class RobotHWImplementation](#class_robothwimplementation)
        * [Methods](#methods)
        * [Fields](#fields)
* [Algorithm](#algorithm)
* [Topics](#topics)
    *[Subscribed](#subscribed)
    *[Publishing](#publishing)
* [Parameters](#parameters)
* [Usage](#usage)


## Project structure
```
.
├── config
│   └── README.md
├── include
│   └── robotec_velocity_controller
│       └── robot_hardware.hpp
├── src
│   ├── controller_node.cpp
│   └── robot_hardware.cpp
├── CMakeLists.txt
├── LICENSE
├── package.xml
├── README.md
└── TODO.md

```

### Folders
* `include` : RobotHWImplementation header file
* `src` : node source code

### Files
* `CHANGELOG.md` : the file where the changes to the project are desribed
* `README.md` : project documentation
* `TODO.md` : suggesting changes in the package 

## Class hierarchy
Class hierarchy is represented as one class, which is an implementation of ros_control's interface. 

### Class RobotHWImplementation
The class is an implementation of the RobotHW class from ros_control package. The class is dedicated to create a filler between ros_control and robot's actual hardware.

#### Methods
* `controllerInitialization` : initialization of the controllers for ControllerManager
* `update` : read, calculate and write data on a time event
* `write` : the method is dedicated to publishing the data when triggered by the timer event
* `(left/right)WheelDegreesCallback()` : callback for receiving degrees data
* `(left/right)WheelvelocityCallback()` : callback for receiving velocity data

#### Fields
* `WHEELS_AMOUNT` : amount of joints to create
* `left_wheel_setpoint_pub` : desirable value of speed to achieve on the left wheel
* `right_wheel_setpoint_pub` : desirable value of speed to achieve on the right wheel
* `_state_interface` : interface for checking state of joints (ros_control)
* `_velocity_interface` : velocity controller interface (ros_control)
* `_positions` : array of positions
* `_velocities` ; array of velocities
* `_efforts` : array of applied efforts
* `_effort_commands` : array of efforts to handle joints control
* `_velocity_commands` : array of velocity to handle joints control
* `_rate` : timer trigger rate 
* `nh_` : ROS node instance
* `timer_event` : ROS timer instance
* `elapsed_time_` : elapsed time since last operation
* `controller_manager_` : controller_manager instance

## Algorithm
This node is an implementation of RobotHW class (ros_control), which is dedicated to be a middleman between high-level operations (e.g. data transferring) and actual joint controllers (ros_controllers). robotec_velocity_controller gets data of which speed it should maintain on two different wheels of differential drive robot. Then, using data about actual state of the wheels, it calculates velocities for them and publishes the data on timer triggers.

## Topics
### Subscribed
* `(left/right)/velocity` : topic with current velocity data
* `(left/right)/degrees` : topic with current degrees data

### Publishing
* `(left/right)/setpoint` : topic with desirable valocity for every wheel separately

## Parameters
* `left_velocity` : name of the left wheel velocity data topic, default : `/left_wheel/velocity`
* `right_velocity` : name of the right wheel velocity data topic, default : `/left_wheel/velocity`
* `left_degrees` : name of the left wheel degrees data topic, default : `/left_wheel/degrees`
* `right_degrees` : name of the right wheel degrees data topic, default : `/right_wheel/degrees`
* `left_setpoint` : name of the left wheel setpoint data topic, default : `/left_wheel/setpoint`
* `right_setpoint` : name of the right wheel setpoint data topic, default : `/right_wheel/setpoint`

## Usage
* The first step of using the package is to create a configuration file. The process is described in config folder

* Next, you should run the command :
```
$ rosrun robotec_velocity_controller velocity_controller
```