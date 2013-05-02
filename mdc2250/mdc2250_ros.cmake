macro(build_mdc2250)
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

if($ENV{ROVER_1})
ADD_DEFINITIONS(-DROVER_1)
endif($ENV{ROVER_1})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Use clang if available
IF(EXISTS /usr/bin/clang)
  set(CMAKE_CXX_COMPILER /usr/bin/clang++)
  set(CMAKE_OSX_DEPLOYMENT_TARGET "")
  set(CMAKE_CXX_FLAGS "-ferror-limit=5")
  set(CMAKE_BUILD_TYPE Debug)
ENDIF(EXISTS /usr/bin/clang)

include_directories(include)

set(MDC2250_SRCS src/mdc2250.cc)

rosbuild_genmsg()
rosbuild_gensrv()

# Build the mdc2250 library
rosbuild_add_library(${PROJECT_NAME} ${MDC2250_SRCS})

# Add boost dependencies
rosbuild_add_boost_directories()
rosbuild_link_boost(${PROJECT_NAME} system filesystem thread)

# Build example
rosbuild_add_executable(mdc2250_example examples/mdc2250_example.cc)
target_link_libraries(mdc2250_example ${PROJECT_NAME})

# Build quad node
rosbuild_add_executable(quad_drive_node src/quad_drive_node.cpp)
target_link_libraries(quad_drive_node ${PROJECT_NAME})

endmacro(build_mdc2250)
