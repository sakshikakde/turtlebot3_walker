cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_walker)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  roslaunch
  tf
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(walker_node src/walker.cpp src/obstacle_detector.cpp src/walker_node.cpp )
target_link_libraries(walker_node ${catkin_LIBRARIES})


# # if(CATKIN_ENABLE_TESTING)
# find_package(rostest REQUIRED)
# add_rostest_gtest(tests_mynode test/test_broadcaster.launch test/tf_broadcaster_test.cpp)
# target_link_libraries(tests_mynode ${catkin_LIBRARIES})
# # endif()
