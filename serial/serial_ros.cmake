macro(build_serial)

  cmake_minimum_required(VERSION 2.8.3)
  project(serial)
  # Load catkin and all dependencies required for this package
  # TODO: remove all from COMPONENTS that are not catkin packages.
  find_package(catkin REQUIRED)

  set(ROS_ROOT $ENV{ROS_ROOT})

  set(SERIAL_SRCS src/serial.cc)
  if(UNIX)
    list(APPEND SERIAL_SRCS src/impl/unix.cc)
  else(UNIX)
    list(APPEND SERIAL_SRCS src/impl/winows.cc)
  endif(UNIX)


  # Collect Link Libraries
  set(SERIAL_LINK_LIBS ${PROJECT_NAME})
  if(UNIX AND NOT APPLE)
    list(APPEND SERIAL_LINK_LIBS rt pthread util)
  endif(UNIX AND NOT APPLE)

  catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   DEPENDS ${SERIAL_LINK_LIBS}
  )
  include_directories(include ${catkin_INCLUDE_DIRS})

  # Build the serial library
  add_library(${PROJECT_NAME} ${SERIAL_SRCS})

  # Build example
  add_executable(serial_example examples/serial_example.cc)
  target_link_libraries(serial_example ${SERIAL_LINK_LIBS} ${catkin_LIBRARIES})

  # Create unit tests
  catkin_add_gtest(serial_tests tests/serial_tests.cc)
  target_link_libraries(serial_tests ${SERIAL_LINK_LIBS} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

endmacro(build_serial)
