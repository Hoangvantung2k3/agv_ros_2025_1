# Install script for directory: /home/tung/agv_ros_2025_1/src/4_wheel_simulation/navigation_msgs/move_base_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/tung/agv_ros_2025_1/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/move_base_msgs/msg" TYPE FILE FILES "/home/tung/agv_ros_2025_1/src/4_wheel_simulation/navigation_msgs/move_base_msgs/msg/RecoveryStatus.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/move_base_msgs/action" TYPE FILE FILES "/home/tung/agv_ros_2025_1/src/4_wheel_simulation/navigation_msgs/move_base_msgs/action/MoveBase.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/move_base_msgs/msg" TYPE FILE FILES
    "/home/tung/agv_ros_2025_1/devel/share/move_base_msgs/msg/MoveBaseAction.msg"
    "/home/tung/agv_ros_2025_1/devel/share/move_base_msgs/msg/MoveBaseActionGoal.msg"
    "/home/tung/agv_ros_2025_1/devel/share/move_base_msgs/msg/MoveBaseActionResult.msg"
    "/home/tung/agv_ros_2025_1/devel/share/move_base_msgs/msg/MoveBaseActionFeedback.msg"
    "/home/tung/agv_ros_2025_1/devel/share/move_base_msgs/msg/MoveBaseGoal.msg"
    "/home/tung/agv_ros_2025_1/devel/share/move_base_msgs/msg/MoveBaseResult.msg"
    "/home/tung/agv_ros_2025_1/devel/share/move_base_msgs/msg/MoveBaseFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/move_base_msgs/cmake" TYPE FILE FILES "/home/tung/agv_ros_2025_1/build/4_wheel_simulation/navigation_msgs/move_base_msgs/catkin_generated/installspace/move_base_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/tung/agv_ros_2025_1/devel/include/move_base_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/tung/agv_ros_2025_1/devel/share/roseus/ros/move_base_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/tung/agv_ros_2025_1/devel/share/common-lisp/ros/move_base_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/tung/agv_ros_2025_1/devel/share/gennodejs/ros/move_base_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/tung/agv_ros_2025_1/devel/lib/python3/dist-packages/move_base_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/tung/agv_ros_2025_1/devel/lib/python3/dist-packages/move_base_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/tung/agv_ros_2025_1/build/4_wheel_simulation/navigation_msgs/move_base_msgs/catkin_generated/installspace/move_base_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/move_base_msgs/cmake" TYPE FILE FILES "/home/tung/agv_ros_2025_1/build/4_wheel_simulation/navigation_msgs/move_base_msgs/catkin_generated/installspace/move_base_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/move_base_msgs/cmake" TYPE FILE FILES
    "/home/tung/agv_ros_2025_1/build/4_wheel_simulation/navigation_msgs/move_base_msgs/catkin_generated/installspace/move_base_msgsConfig.cmake"
    "/home/tung/agv_ros_2025_1/build/4_wheel_simulation/navigation_msgs/move_base_msgs/catkin_generated/installspace/move_base_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/move_base_msgs" TYPE FILE FILES "/home/tung/agv_ros_2025_1/src/4_wheel_simulation/navigation_msgs/move_base_msgs/package.xml")
endif()

