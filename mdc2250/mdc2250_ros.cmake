macro(build_mdc2250)
cmake_minimum_required(VERSION 2.4.6)

project(mdc2250)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp rospy tf geometry_msgs nav_msgs serial serial_utils message_generation)
find_package(Boost REQUIRED COMPONENTS thread system)

if($ENV{ROVER_1})
ADD_DEFINITIONS(-DROVER_1)
endif($ENV{ROVER_1})

add_message_files(
  FILES
  Encoders.msg
  MotorRaw.msg
  StampedEncoders.msg
)
add_service_files(
  FILES
  estop.srv
  setspeed.srv
)

generate_messages(
   DEPENDENCIES std_msgs geometry_msgs
)
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy tf geometry_msgs nav_msgs serial serial_utils message_runtime
   LIBRARIES serial serial_utils
)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} include)

set(MDC2250_SRCS src/mdc2250.cc)

# Build the mdc2250 library
add_library(mdc2250 ${MDC2250_SRCS})
target_link_libraries(mdc2250 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(mdc2250 ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_generate_services_cpp)

# Build quad node
add_executable(quad_drive_node src/quad_drive_node.cpp)
target_link_libraries(quad_drive_node mdc2250 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(teleop_joy src/teleop_joy.cpp)
target_link_libraries(teleop_joy ${catkin_LIBRARIES})

install(TARGETS quad_drive_node teleop_joy
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)
install(TARGETS mdc2250
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

endmacro(build_mdc2250)
