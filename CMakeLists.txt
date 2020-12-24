cmake_minimum_required(VERSION 2.8.3)
project(robotec_velocity_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	controller_manager
	roscpp
	std_msgs
	sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

## Include essential directories
include_directories(
 include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_executable(velocity_controller src/robot_hardware.cpp src/controller_node.cpp)
add_dependencies(velocity_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocity_controller ${catkin_LIBRARIES})
