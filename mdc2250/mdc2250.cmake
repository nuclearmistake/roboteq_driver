macro(build_mdc2250)
## Project Setup
cmake_minimum_required(VERSION 2.4.6)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(mdc2250)

## Configurations

# Use clang if available
IF(EXISTS /usr/bin/clang)
  set(CMAKE_CXX_COMPILER /usr/bin/clang++)
  set(CMAKE_OSX_DEPLOYMENT_TARGET "")
  set(CMAKE_CXX_FLAGS "-ferror-limit=5")
  set(CMAKE_BUILD_TYPE Debug)
ENDIF(EXISTS /usr/bin/clang)

option(MDC2250_BUILD_TESTS "Build all of the mdc2250 tests." OFF)
option(MDC2250_BUILD_EXAMPLES "Build all of the mdc2250 examples." OFF)

# Allow for building shared libs override
IF(NOT BUILD_SHARED_LIBS)
    set(BUILD_SHARED_LIBS OFF)
ENDIF(NOT BUILD_SHARED_LIBS)

# Set the default path for built executables to the "bin" directory
IF(NOT DEFINED(EXECUTABLE_OUTPUT_PATH))
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
ENDIF(NOT DEFINED(EXECUTABLE_OUTPUT_PATH))
# set the default path for built libraries to the "lib" directory
IF(NOT DEFINED(LIBRARY_OUTPUT_PATH))
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
ENDIF(NOT DEFINED(LIBRARY_OUTPUT_PATH))

## Configure the build system

# Add the include folder to the include path
include_directories(${PROJECT_SOURCE_DIR}/include)

# Add default source files
set(MDC2250_SRCS src/mdc2250.cc)
# Add default header files
set(MDC2250_HEADERS include/mdc2250/mdc2250.h)

# Find Boost, if it hasn't already been found
IF(NOT Boost_FOUND OR NOT Boost_SYSTEM_FOUND OR NOT Boost_FILESYSTEM_FOUND OR NOT Boost_THREAD_FOUND)
    find_package(Boost COMPONENTS system filesystem thread REQUIRED)
ENDIF(NOT Boost_FOUND OR NOT Boost_SYSTEM_FOUND OR NOT Boost_FILESYSTEM_FOUND OR NOT Boost_THREAD_FOUND)

link_directories(${Boost_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

set(MDC2250_LINK_LIBS ${Boost_SYSTEM_LIBRARY}
                      ${Boost_FILESYSTEM_LIBRARY}
                      ${Boost_THREAD_LIBRARY})

# Find serial, if it hasn't already been found
IF(NOT serial_FOUND)
    find_package(serial REQUIRED)
ENDIF(NOT serial_FOUND)

if(serial_FOUND)
  include_directories(${serial_INCLUDE_DIRS})
  list(APPEND MDC2250_LINK_LIBS ${serial_LIBRARIES})
else(serial_FOUND)
  message(FATAL_ERROR "Serial library was not found.")
endif(serial_FOUND)

## Build the mdc2250 Library

# Compile the Library
add_library(mdc2250 ${MDC2250_SRCS} ${MDC2250_HEADERS})
target_link_libraries(mdc2250 ${MDC2250_LINK_LIBS})

## Build Examples

# If asked to
IF(MDC2250_BUILD_EXAMPLES)
    # Compile the mdc2250 Test program
    add_executable(mdc2250_example examples/mdc2250_example.cc)
    # Link the Test program to the mdc2250 library
    target_link_libraries(mdc2250_example mdc2250)
ENDIF(MDC2250_BUILD_EXAMPLES)

## Build tests

# If asked to
IF(MDC2250_BUILD_TESTS)
    # Find Google Test
    enable_testing()
    find_package(GTest REQUIRED)
    include_directories(${GTEST_INCLUDE_DIRS})

    # Compile the mdc2250 Test program
    add_executable(mdc2250_tests tests/mdc2250_tests.cc)
    # Link the Test program to the mdc2250 library
    target_link_libraries(mdc2250_tests ${GTEST_BOTH_LIBRARIES}
                          mdc2250)

    add_test(AllTestsIntest_mdc2250 mdc2250_tests)
ENDIF(MDC2250_BUILD_TESTS)

## Setup install and uninstall

# Unless asked not to...
IF(NOT MDC2250_DONT_CONFIGURE_INSTALL)
    # Configure make install
    IF(NOT CMAKE_INSTALL_PREFIX)
        SET(CMAKE_INSTALL_PREFIX /usr/local)
    ENDIF(NOT CMAKE_INSTALL_PREFIX)
    
    INSTALL(TARGETS mdc2250
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib
    )
    
    INSTALL(FILES include/mdc2250/mdc2250.h
            DESTINATION include/mdc2250)
    
    IF(NOT CMAKE_FIND_INSTALL_PATH)
        set(CMAKE_FIND_INSTALL_PATH ${CMAKE_ROOT})
    ENDIF(NOT CMAKE_FIND_INSTALL_PATH)
    
    INSTALL(FILES Findmdc2250.cmake DESTINATION ${CMAKE_FIND_INSTALL_PATH}/Modules/)
    
    ADD_CUSTOM_TARGET(uninstall @echo uninstall package)
    
    IF (UNIX)
      ADD_CUSTOM_COMMAND(
        COMMENT "uninstall package"
        COMMAND xargs ARGS rm < install_manifest.txt
        
        TARGET  uninstall
      )
    ELSE(UNIX)
      ADD_CUSTOM_COMMAND(
        COMMENT "uninstall only implemented in unix"
        TARGET  uninstall
      )
    ENDIF(UNIX)
ENDIF(NOT MDC2250_DONT_CONFIGURE_INSTALL)
endmacro(build_mdc2250)
