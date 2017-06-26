cmake_minimum_required(VERSION 2.8.3)
project(ur10_gripper)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    std_msgs
    message_generation
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  CATKIN_DEPENDS
    ur10_gripper_msgs
    std_msgs
    moveit_core
    moveit_ros_planning_interface
    message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(ur10_gripper_server src/ur10_gripper_server.cpp)
target_link_libraries(ur10_gripper_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ur10_gripper_server DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

