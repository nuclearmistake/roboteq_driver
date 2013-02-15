cmake_minimum_required(VERSION 2.4.6)

set(ROS_ROOT $ENV{ROS_ROOT})

option(MDC2250_BUILD_WIHOUT_ROS "Build without ROS?" OFF)

if(DEFINED ROS_ROOT AND NOT MDC2250_BUILD_WIHOUT_ROS)
  # Build with ROS
  include(mdc2250_ros.cmake)
  message("Building with ROS")
  build_mdc2250()
else(DEFINED ROS_ROOT AND NOT MDC2250_BUILD_WIHOUT_ROS)
  # Build normally
  include(mdc2250.cmake)
  message("Building stand alone")
  build_mdc2250()
endif(DEFINED ROS_ROOT AND NOT MDC2250_BUILD_WIHOUT_ROS)
