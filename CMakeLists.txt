cmake_minimum_required(VERSION 3.0.2)
project(algorithm_study)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  std_msgs
)
find_package(OpenCV REQUIRED)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES algorithm_study
 CATKIN_DEPENDS cv_bridge roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(global_planner src/main.cpp src/Dijkstra.cpp)
target_link_libraries(global_planner ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
