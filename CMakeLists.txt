cmake_minimum_required(VERSION 2.8.3)
project(omnivelma_2d_nav)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  move_base_msgs
  nav_msgs
  omnivelma_msgs
  roscpp
  sensor_msgs
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  # INCLUDE_DIRS include
#  LIBRARIES omnivelma_2d_nav
  CATKIN_DEPENDS actionlib_msgs move_base_msgs nav_msgs omnivelma_msgs roscpp sensor_msgs tf tf2 tf2_ros
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(omnivelma_speed_broadcaster src/omnivelma_speed_broadcaster.cpp)
target_link_libraries(omnivelma_speed_broadcaster ${catkin_LIBRARIES})

add_executable(laser_tf_broadcaster src/laser_tf_broadcaster.cpp)
target_link_libraries(laser_tf_broadcaster ${catkin_LIBRARIES})

add_executable(camera_tf_broadcaster src/camera_tf_broadcaster.cpp)
target_link_libraries(camera_tf_broadcaster ${catkin_LIBRARIES})

add_executable(odom_noise src/odom_noise.cpp)
target_link_libraries(odom_noise ${catkin_LIBRARIES})

add_executable(message_cov src/message_cov.cpp)
target_link_libraries(message_cov ${catkin_LIBRARIES})

add_executable(simple_navigation_goals src/simple_navigation_goals.cpp)
target_link_libraries(simple_navigation_goals ${catkin_LIBRARIES})

add_executable(odometry src/odometry.cpp)
target_link_libraries(odometry ${catkin_LIBRARIES})