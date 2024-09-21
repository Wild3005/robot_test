cmake_minimum_required(VERSION 3.0.2)
project(main_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_path(LIBUSBP_INCLUDE_DIR
        NAMES libusbp.hpp
        PATH_SUFFIXES "include" "libusbp" "libusbp-1")
find_library(LIBUSBP_LIBRARY
        NAMES libusbp-1.so)

message(STATUS "LIBUSBP_INCLUDE_DIR: ${LIBUSBP_INCLUDE_DIR}")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  sensor_msgs
  roscpp
  roslib
  std_msgs
  msg_pkg
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs msg_pkg
)

###########
## Build ##
###########

## Declare a C++ executable
add_executable(main_node src/main_node/controller.cpp)
add_executable(node_serial src/node_serial/serial.cpp)

include_directories (
  "${CMAKE_SOURCE_DIR}/include"
)

## Specify additional locations of header files
target_include_directories(node_serial PRIVATE
  ${LIBUSBP_INCLUDE_DIR}
  include
)

## Specify libraries to link a library or executable target against
target_link_libraries(node_serial PRIVATE
  ${catkin_LIBRARIES}
  ${LIBUSBP_LIBRARY}
)

target_link_libraries(main_node 
${catkin_LIBRARIES}
${roslib_LIBRARIES}
)

## Add dependencies
add_dependencies(main_node msg_pkg_generate_messages_cpp)
add_dependencies(node_serial msg_pkg_generate_messages_cpp)
## Mark executables for installation
# install(TARGETS ${PROJECT_NAME}_node node_serial
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_main_pkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
