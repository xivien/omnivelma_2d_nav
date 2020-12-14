cmake_minimum_required(VERSION 2.8.3)
project(omnivelma_nav_setup)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  omnivelma_msgs
  roscpp
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS nav_msgs omnivelma_msgs roscpp tf tf2 tf2_ros
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(omnivelma_speed_broadcaster src/omnivelma_speed_broadcaster.cpp)
target_link_libraries(omnivelma_speed_broadcaster ${catkin_LIBRARIES})

add_executable(odometry src/odometry.cpp)
target_link_libraries(odometry ${catkin_LIBRARIES})
