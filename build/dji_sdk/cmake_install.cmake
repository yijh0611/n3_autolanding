# Install script for directory: /home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/aims/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/aims/catkin_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/aims/catkin_ws/install" TYPE PROGRAM FILES "/home/aims/catkin_ws/build/dji_sdk/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/aims/catkin_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/aims/catkin_ws/install" TYPE PROGRAM FILES "/home/aims/catkin_ws/build/dji_sdk/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/aims/catkin_ws/install/setup.bash;/home/aims/catkin_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/aims/catkin_ws/install" TYPE FILE FILES
    "/home/aims/catkin_ws/build/dji_sdk/catkin_generated/installspace/setup.bash"
    "/home/aims/catkin_ws/build/dji_sdk/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/aims/catkin_ws/install/setup.sh;/home/aims/catkin_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/aims/catkin_ws/install" TYPE FILE FILES
    "/home/aims/catkin_ws/build/dji_sdk/catkin_generated/installspace/setup.sh"
    "/home/aims/catkin_ws/build/dji_sdk/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/aims/catkin_ws/install/setup.zsh;/home/aims/catkin_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/aims/catkin_ws/install" TYPE FILE FILES
    "/home/aims/catkin_ws/build/dji_sdk/catkin_generated/installspace/setup.zsh"
    "/home/aims/catkin_ws/build/dji_sdk/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/aims/catkin_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/aims/catkin_ws/install" TYPE FILE FILES "/home/aims/catkin_ws/build/dji_sdk/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/aims/catkin_ws/build/dji_sdk/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/msg" TYPE FILE FILES
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/msg/FCTimeInUTC.msg"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/msg/GPSUTC.msg"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/msg/Gimbal.msg"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/msg/Waypoint.msg"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/msg/WaypointList.msg"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/msg/MobileData.msg"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/msg/PayloadData.msg"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/msg/MissionWaypointAction.msg"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/msg/MissionWaypoint.msg"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/msg/MissionWaypointTask.msg"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/msg/MissionHotpointTask.msg"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/msg/FlightAnomaly.msg"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/msg/VOPosition.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/srv" TYPE FILE FILES
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/Activation.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/CameraAction.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/DroneTaskControl.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/SDKControlAuthority.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/SetLocalPosRef.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/MFIOConfig.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/MFIOSetValue.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/DroneArmControl.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/MissionStatus.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/MissionWpAction.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/MissionHpAction.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/MissionWpUpload.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/MissionWpSetSpeed.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/MissionWpGetSpeed.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/MissionWpGetInfo.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/MissionHpUpload.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/MissionHpGetInfo.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/MissionHpUpdateYawRate.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/MissionHpUpdateRadius.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/MissionHpResetYaw.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/SendMobileData.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/SendPayloadData.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/SetHardSync.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/QueryDroneVersion.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/Stereo240pSubscription.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/StereoVGASubscription.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/StereoDepthSubscription.srv"
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/SetupCameraStream.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/cmake" TYPE FILE FILES "/home/aims/catkin_ws/build/dji_sdk/catkin_generated/installspace/dji_sdk-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/aims/catkin_ws/devel/.private/dji_sdk/include/dji_sdk")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/aims/catkin_ws/devel/.private/dji_sdk/share/roseus/ros/dji_sdk")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/aims/catkin_ws/devel/.private/dji_sdk/share/common-lisp/ros/dji_sdk")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/aims/catkin_ws/devel/.private/dji_sdk/share/gennodejs/ros/dji_sdk")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/aims/catkin_ws/devel/.private/dji_sdk/lib/python2.7/dist-packages/dji_sdk")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/aims/catkin_ws/devel/.private/dji_sdk/lib/python2.7/dist-packages/dji_sdk" REGEX "/\\_\\_init\\_\\_\\.py$" EXCLUDE REGEX "/\\_\\_init\\_\\_\\.pyc$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/aims/catkin_ws/devel/.private/dji_sdk/lib/python2.7/dist-packages/dji_sdk" FILES_MATCHING REGEX "/home/aims/catkin_ws/devel/.private/dji_sdk/lib/python2.7/dist-packages/dji_sdk/.+/__init__.pyc?$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/aims/catkin_ws/build/dji_sdk/catkin_generated/installspace/dji_sdk.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/cmake" TYPE FILE FILES "/home/aims/catkin_ws/build/dji_sdk/catkin_generated/installspace/dji_sdk-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/cmake" TYPE FILE FILES
    "/home/aims/catkin_ws/build/dji_sdk/catkin_generated/installspace/dji_sdkConfig.cmake"
    "/home/aims/catkin_ws/build/dji_sdk/catkin_generated/installspace/dji_sdkConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk" TYPE FILE FILES "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dji_sdk" TYPE DIRECTORY FILES "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/include/dji_sdk/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dji_sdk" TYPE EXECUTABLE FILES "/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/dji_sdk_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node"
         OLD_RPATH "/usr/local/cuda/lib64:/opt/ros/melodic/lib:/usr/local/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/launch" TYPE DIRECTORY FILES "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/launch")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dji_sdk" TYPE PROGRAM FILES "/home/aims/catkin_ws/build/dji_sdk/catkin_generated/installspace/gimbal_control.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dji_sdk" TYPE PROGRAM FILES "/home/aims/catkin_ws/build/dji_sdk/catkin_generated/installspace/talker.py")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/aims/catkin_ws/build/dji_sdk/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/aims/catkin_ws/build/dji_sdk/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
