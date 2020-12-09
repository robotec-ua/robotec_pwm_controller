cmake_minimum_required(VERSION 2.8.3)
project(driver_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	controller_manager
	roscpp
	std_msgs
	sensor_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################

##s
catkin_package()

###########
## Build ##
###########

## Include essential directories
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(2WControl src/RobotHWImplementation.cpp src/DriverControlNode.cpp)
add_dependencies(2WControl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(2WControl ${catkin_LIBRARIES})
