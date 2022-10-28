cmake_minimum_required(VERSION 2.8.3)
project(dji_sdk)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") # https://answers.ros.org/question/354932/couldnt-find-executable/

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  # --- OpenCV ---
  cmake_modules
  cv_bridge
  image_geometry
  image_transport
  nodelet
  pluginlib
  tf
)
# --- OpenCV ---
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)
# --- ---

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake-modules")
find_package(djiosdk-core QUIET)
if(NOT djiosdk-core_FOUND)
  MESSAGE(STATUS "DO NOT FIND THE djiosdk-core LIB, WILL TRY TO DOWNLOAD FORM GITHUB, COMPILE, INSTALL")
  execute_process(COMMAND mkdir ${CMAKE_CURRENT_SOURCE_DIR}/../cmake-modules/build)
  execute_process(
          COMMAND cmake ..
          WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../cmake-modules/build)

  execute_process(
          COMMAND make
          WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../cmake-modules/build)
endif()

find_package(DJIOSDK REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  FCTimeInUTC.msg
  GPSUTC.msg
  Gimbal.msg
  Waypoint.msg
  WaypointList.msg
  MobileData.msg
  PayloadData.msg
  MissionWaypointAction.msg
  MissionWaypoint.msg
  MissionWaypointTask.msg
  MissionHotpointTask.msg
  FlightAnomaly.msg
  VOPosition.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Activation.srv
  CameraAction.srv
  DroneTaskControl.srv
  SDKControlAuthority.srv
  SetLocalPosRef.srv
  MFIOConfig.srv
  MFIOSetValue.srv
  DroneArmControl.srv
  MissionStatus.srv
  MissionWpAction.srv
  MissionHpAction.srv
  MissionWpUpload.srv
  MissionWpSetSpeed.srv
  MissionWpGetSpeed.srv
  MissionWpGetInfo.srv
  MissionHpUpload.srv
  MissionHpGetInfo.srv
  MissionHpUpdateYawRate.srv
  MissionHpUpdateRadius.srv
  MissionHpResetYaw.srv
  SendMobileData.srv
  SendPayloadData.srv
  SetHardSync.srv
  QueryDroneVersion.srv
  Stereo240pSubscription.srv
  StereoVGASubscription.srv
  StereoDepthSubscription.srv
  SetupCameraStream.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    message_runtime
    geometry_msgs
    nav_msgs
    std_msgs
    nmea_msgs
    # --- OpenCV ---
    cv_bridge
    image_transport
    nodelet
    pluginlib
    roscpp
    sensor_msgs
    tf
  DEPENDS
    OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${DJIOSDK_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(dji_sdk
#   src/${PROJECT_NAME}/dji_sdk.cpp
# )

## Declare a cpp executable
add_executable(dji_sdk_node
  src/main.cpp
  src/modules/dji_sdk_node_control.cpp
  src/modules/dji_sdk_node_services.cpp
  src/modules/dji_sdk_node.cpp
  src/modules/dji_sdk_node_mission_services.cpp
  src/modules/dji_sdk_node_subscriber.cpp
  src/modules/dji_sdk_node_publisher.cpp
  src/modules/dji_sdk_node_mobile_comm.cpp
  src/modules/dji_sdk_node_payload_comm.cpp
  src/modules/dji_sdk_node_time_sync.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(dji_sdk_node
  dji_sdk_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(dji_sdk_node
  ${catkin_LIBRARIES}
  ${DJIOSDK_LIBRARIES}
  ${OpenCV_LIBS}
)

#############
## Install ##
#############
# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS dji_sdk_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# python codes
catkin_install_python(PROGRAMS scripts/gimbal_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/talker.py
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )



#add_executable(dji_sdk_node_control src/dji_sdk_node_control.cpp)
#target_link_libraries(dji_sdk_node_control ${catkin_LIBRARIES})
#add_dependencies(dji_sdk_node_control dji_sdk_generate_messages_cpp)

# add_executable(talker_ros src/talker_ros.cpp)
# target_link_libraries(talker_ros ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
# add_dependencies(talker_ros dji_sdk_generate_messages_cpp)

# add_executable(talker_ros_1 src/talker_ros_1.cpp)
# target_link_libraries(talker_ros_1 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
# add_dependencies(talker_ros_1 dji_sdk_generate_messages_cpp)

# add_executable(talker_ros_2 src/talker_ros_2.cpp)
# target_link_libraries(talker_ros_2 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
# add_dependencies(talker_ros_2 dji_sdk_generate_messages_cpp)

# add_executable(talker_ros_3 src/talker_ros_3.cpp)
# target_link_libraries(talker_ros_3 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
# add_dependencies(talker_ros_3 dji_sdk_generate_messages_cpp)

# add_executable(talker_ros_4 src/talker_ros_4.cpp)
# target_link_libraries(talker_ros_4 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
# add_dependencies(talker_ros_4 dji_sdk_generate_messages_cpp)

# add_executable(talker_ros_5 src/talker_ros_5.cpp)
# target_link_libraries(talker_ros_5 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
# add_dependencies(talker_ros_5 dji_sdk_generate_messages_cpp)

# add_executable(talker_ros_6 src/talker_ros_6.cpp)
# target_link_libraries(talker_ros_6 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
# add_dependencies(talker_ros_6 dji_sdk_generate_messages_cpp)

# add_executable(talker_ros_7 src/talker_ros_7.cpp)
# target_link_libraries(talker_ros_7 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})

# add_executable(talker_ros_8 src/talker_ros_8.cpp)
# target_link_libraries(talker_ros_8 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
# add_dependencies(talker_ros_8 dji_sdk_generate_messages_cpp)

# add_executable(talker_ros_9 src/talker_ros_9.cpp)
# target_link_libraries(talker_ros_9 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
# add_dependencies(talker_ros_9 dji_sdk_generate_messages_cpp)

# add_executable(talker_ros_10 src/talker_ros_10.cpp)
# target_link_libraries(talker_ros_10 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
# add_dependencies(talker_ros_10 dji_sdk_generate_messages_cpp)

# add_executable(talker_ros_11 src/talker_ros_11.cpp)
# target_link_libraries(talker_ros_11 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
# add_dependencies(talker_ros_11 dji_sdk_generate_messages_cpp)

# add_executable(talker_ros_12 src/talker_ros_12.cpp)
# target_link_libraries(talker_ros_12 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
# add_dependencies(talker_ros_12 dji_sdk_generate_messages_cpp)

# add_executable(talker_ros_13 src/talker_ros_13.cpp)
# target_link_libraries(talker_ros_13 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
# add_dependencies(talker_ros_13 dji_sdk_generate_messages_cpp)

# add_executable(talker_ros_14 src/talker_ros_14.cpp)
# target_link_libraries(talker_ros_14 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
# add_dependencies(talker_ros_14 dji_sdk_generate_messages_cpp)

# add_executable(talker_ros_15 src/talker_ros_15.cpp)
# target_link_libraries(talker_ros_15 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
# add_dependencies(talker_ros_15 dji_sdk_generate_messages_cpp)

# add_executable(talker_ros_16 src/talker_ros_16.cpp)
# target_link_libraries(talker_ros_16 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
# add_dependencies(talker_ros_16 dji_sdk_generate_messages_cpp)

# add_executable(talker_ros_17 src/talker_ros_17.cpp)
# target_link_libraries(talker_ros_17 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
# add_dependencies(talker_ros_17 dji_sdk_generate_messages_cpp)

# add_executable(talker_ros_18 src/talker_ros_18.cpp)
# target_link_libraries(talker_ros_18 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
# add_dependencies(talker_ros_18 dji_sdk_generate_messages_cpp)

# add_executable(talker_ros_19 src/talker_ros_19.cpp)
# target_link_libraries(talker_ros_19 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
# add_dependencies(talker_ros_19 dji_sdk_generate_messages_cpp)

add_executable(talker_ros_20 src/talker_ros_20.cpp)
target_link_libraries(talker_ros_20 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
add_dependencies(talker_ros_20 dji_sdk_generate_messages_cpp)

add_executable(talker_ros_21 src/talker_ros_21.cpp)
target_link_libraries(talker_ros_21 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
add_dependencies(talker_ros_21 dji_sdk_generate_messages_cpp)

add_executable(talker_ros_22 src/talker_ros_22.cpp)
target_link_libraries(talker_ros_22 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
add_dependencies(talker_ros_22 dji_sdk_generate_messages_cpp)

add_executable(land_1 src/land_1.cpp)
target_link_libraries(land_1 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
add_dependencies(land_1 dji_sdk_generate_messages_cpp)



add_executable(talker_sim_1 src/talker_sim_1.cpp)
target_link_libraries(talker_sim_1 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
add_dependencies(talker_sim_1 dji_sdk_generate_messages_cpp)

add_executable(talker_sim_2 src/talker_sim_2.cpp)
target_link_libraries(talker_sim_2 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
add_dependencies(talker_sim_2 dji_sdk_generate_messages_cpp)

add_executable(talker_sim_3 src/talker_sim_3.cpp)
target_link_libraries(talker_sim_3 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
add_dependencies(talker_sim_3 dji_sdk_generate_messages_cpp)

add_executable(talker_sim_4 src/talker_sim_4.cpp)
target_link_libraries(talker_sim_4 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
add_dependencies(talker_sim_4 dji_sdk_generate_messages_cpp)




add_executable(platform_simulator src/platform_simulator.cpp)
target_link_libraries(platform_simulator ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
add_dependencies(platform_simulator dji_sdk_generate_messages_cpp)

add_executable(get_tf src/get_tf.cpp)
target_link_libraries(get_tf ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
add_dependencies(get_tf dji_sdk_generate_messages_cpp)





# add_executable(listener_ros src/listener_ros.cpp)
# target_link_libraries(listener_ros ${catkin_LIBRARIES})
# add_dependencies(listener_ros dji_sdk_generate_messages_cpp)

# add_executable(talker_ros_original src/talker_ros_original.cpp)
# target_link_libraries(talker_ros_original ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES} ${APRILTAG_ROS})
# add_dependencies(talker_ros_original dji_sdk_generate_messages_cpp)


# add_executable(demo_flight_control_copy
# 		src/demo_flight_control_copy.cpp)

# target_link_libraries(demo_flight_control_copy
# 		${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES}
# )
# add_dependencies(demo_flight_control_copy dji_sdk_generate_messages_cpp)


# add_executable(dji_sdk src/opencv_tst.cpp)
# target_link_libraries(dji_sdk
# 		${catkin_LIBRARIES}
#     ${DJIOSDK_LIBRARIES}
#     ${OpenCV_LIBRARIES}
# )
# add_dependencies(dji_sdk dji_sdk_generate_messages_cpp)


# add_executable(opencv_tst src/opencv_tst.cpp)
# target_link_libraries(opencv_tst
# 		${catkin_LIBRARIES}
#     ${DJIOSDK_LIBRARIES}
#     ${OpenCV_LIBRARIES}
# )
# add_dependencies(opencv_tst dji_sdk_generate_messages_cpp)



add_executable(test_code src/test_code.cpp)
target_link_libraries(test_code ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
add_dependencies(test_code dji_sdk_generate_messages_cpp)




add_executable(camera_to_tf src/camera_to_tf.cpp)
target_link_libraries(camera_to_tf ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES} apriltag::apriltag)
add_dependencies(camera_to_tf dji_sdk_generate_messages_cpp)

add_executable(camera_pub src/camera_pub.cpp)
target_link_libraries(camera_pub ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES} apriltag::apriltag)
add_dependencies(camera_pub dji_sdk_generate_messages_cpp)

add_executable(camera_pub_no_imshow src/camera_pub_no_imshow.cpp)
target_link_libraries(camera_pub_no_imshow ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES} apriltag::apriltag)
add_dependencies(camera_pub_no_imshow dji_sdk_generate_messages_cpp)

add_executable(camera_pub_and_apriltag src/camera_pub_and_apriltag.cpp)
target_link_libraries(camera_pub_and_apriltag ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES} apriltag::apriltag)
add_dependencies(camera_pub_and_apriltag dji_sdk_generate_messages_cpp)

add_executable(chk_gimbal src/chk_gimbal.cpp)
target_link_libraries(chk_gimbal ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
add_dependencies(chk_gimbal dji_sdk_generate_messages_cpp)

add_executable(save_log src/save_log.cpp)
target_link_libraries(save_log ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
add_dependencies(save_log dji_sdk_generate_messages_cpp)

add_executable(save_log_100 src/save_log_100.cpp)
target_link_libraries(save_log_100 ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
add_dependencies(save_log_100 dji_sdk_generate_messages_cpp)

add_executable(save_log_sim src/save_log_sim.cpp)
target_link_libraries(save_log_sim ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
add_dependencies(save_log_sim dji_sdk_generate_messages_cpp)

add_executable(save_log_platform src/save_log_platform.cpp)
target_link_libraries(save_log_platform ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
add_dependencies(save_log_platform dji_sdk_generate_messages_cpp)

add_executable(calculate_vel src/calculate_vel.cpp)
target_link_libraries(calculate_vel ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})
add_dependencies(calculate_vel dji_sdk_generate_messages_cpp)
