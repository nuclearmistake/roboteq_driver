macro(build_serial_utils)

project(serial_utils)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS serial)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

set(SERIAL_UTILS_SRCS src/serial_listener.cc)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS serial
   LIBRARIES ${PROJECT_NAME}
)
include_directories(${catkin_INCLUDE_DIRS} include)

# Build the serial library
add_library(${PROJECT_NAME} ${SERIAL_UTILS_SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

endmacro(build_serial_utils)
