cmake_minimum_required(VERSION 3.0.2)
project(lg)

# Find required packages (no custom message generation)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs  # Standard message types
)

# Declare a catkin package without custom message generation
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs  # Only use the standard ROS dependencies
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Add executable for master_node
add_executable(master_node src/master.cpp)
target_link_libraries(master_node ${catkin_LIBRARIES})

# Add executable for rangefinder
add_executable(rangefinder src/rangefinder.cpp)
target_link_libraries(rangefinder ${catkin_LIBRARIES})

# Add executable for serial node
add_executable(serial src/serial.cpp)
target_link_libraries(serial ${catkin_LIBRARIES})

add_executable(incCalc src/incCalc.cpp)
target_link_libraries(incCalc ${catkin_LIBRARIES})

