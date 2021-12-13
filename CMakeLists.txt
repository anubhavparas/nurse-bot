cmake_minimum_required(VERSION 3.0.2)
project(nurse_bot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  move_base
  move_base_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
  moveit_ros_planning_interface
  tf2
  # tf2_geometry_msgs
  tf2_ros
  control_msgs
  fiducial_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Task.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
add_action_files(
  FILES
  NBTask.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  move_base_msgs   
  sensor_msgs   
  std_msgs
  control_msgs
  fiducial_msgs
  # tf2_geometry_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need 
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs move_base move_base_msgs roscpp rospy sensor_msgs std_msgs tf message_runtime moveit_ros_planning_interface control_msgs fiducial_msgs tf2_ros # tf2_geometry_msgs 
#  LIBRARIES nurse-bot
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/nurse-bot.cpp
# )



## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_ui_node 
              src/user_interface_node.cpp 
              src/task_publisher.cpp
              src/task_action_client.cpp
              src/user_interface.cpp
              )
target_link_libraries(${PROJECT_NAME}_ui_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_ui_node nurse_bot_generate_messages_cpp)

add_executable(${PROJECT_NAME}_task_sub_node 
              src/task_subscriber_node.cpp 
              src/task_subscriber.cpp 
              src/map_navigator.cpp
              src/movebaseaction_wrapper.cpp
              src/task_action.cpp
              )
target_link_libraries(${PROJECT_NAME}_task_sub_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_task_sub_node nurse_bot_generate_messages_cpp)

add_executable(${PROJECT_NAME}_action_server_node 
              src/action_server_node.cpp 
              src/task_action_server.cpp 
              src/map_navigator.cpp
              src/movebaseaction_wrapper.cpp
              src/task_action.cpp
              src/aruco_detector.cpp
              src/arm_controller.cpp
              src/head_controller.cpp
              src/torso_controller.cpp
              src/gripper_controller.cpp
              src/robot_controller.cpp
              src/pick_place_controller.cpp
              )
target_link_libraries(${PROJECT_NAME}_action_server_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_action_server_node nurse_bot_generate_messages_cpp)


#############
## Install ##
#############


#############
## Testing ##
#############



set(gmock_INCLUDE_DIR /usr/include/gmock)
set(gmock_LIB_DIR /usr/lib)

# if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
#   find_package(code_coverage REQUIRED)
#   # Add compiler flags for coverage instrumentation before defining any targets
#   APPEND_COVERAGE_COMPILER_FLAGS()
# endif()


# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
# # We probably don't want this to run on every build.
# option(COVERAGE "Generate Coverage Data" OFF)

# if (COVERAGE)
#     include(CodeCoverage)
#     set(LCOV_REMOVE_EXTRA "'vendor/*'")
#     setup_target_for_coverage(code_coverage test/cpp-test coverage)
#     set(COVERAGE_SRCS src/aruco_detector.cpp include/aruco_detector.hpp)

#     SET(CMAKE_CXX_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage")
#     SET(CMAKE_C_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage")
#     SET(CMAKE_EXE_LINKER_FLAGS "-fprofile-arcs -ftest-coverage")
# else()
#     set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic -g")
# endif()

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -pthread")
	
  add_rostest_gtest(
    nursebot_gtest
    test/nursebot_gtest.test
    test/main.cpp
    test/test_map_navigator.cpp
    test/test_pick_place_controller.cpp
    test/test_robot_controller.cpp
    src/map_navigator.cpp
    src/movebaseaction_wrapper.cpp
    src/pick_place_controller.cpp
    src/robot_controller.cpp
    src/aruco_detector.cpp
    src/arm_controller.cpp
    src/head_controller.cpp
    src/torso_controller.cpp
    src/gripper_controller.cpp
  )
  target_link_libraries(nursebot_gtest gmock ${catkin_LIBRARIES})
  
  
  add_rostest_gtest(
    nursebot_test
    test/nursebot_test.test
    test/main.cpp
    )
  target_link_libraries(nursebot_test gmock ${catkin_LIBRARIES})

  add_rostest_gtest(
    aruco_test
    test/nursebot_aruco_test.test
    # test/main.cpp
    test/test_aruco_detector.cpp
    src/aruco_detector.cpp
  )
  target_link_libraries(aruco_test gmock ${catkin_LIBRARIES})

  add_rostest_gtest(
    controller_test
    test/nursebot_controllers_test.test
    test/main.cpp
    test/test_gripper_controller.cpp
    test/test_head_controller.cpp
    test/test_torso_controller.cpp
    src/gripper_controller.cpp
    src/head_controller.cpp
    src/torso_controller.cpp
  )
  target_link_libraries(controller_test gmock ${catkin_LIBRARIES})

  add_rostest_gtest(
    server_client_test
    test/nursebot_server_client_test.test
    test/main.cpp
    # test/test_task_action_server.cpp
    test/test_movebaseaction_wrapper.cpp
    # src/task_action_server.cpp
    src/task_action_client.cpp
    src/movebaseaction_wrapper.cpp
  )
  target_link_libraries(server_client_test gmock ${catkin_LIBRARIES})

endif()