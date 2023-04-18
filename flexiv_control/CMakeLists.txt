cmake_minimum_required(VERSION 3.5)
project(flexiv_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_FOR_ARM64 "Link to RDK library for arm64 processor, otherwise link to x64" OFF)

find_package(catkin REQUIRED COMPONENTS
  # flexiv_msgs
  roscpp
  rospy
  std_msgs
  urdf
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    # flexiv_msgs
    roscpp
    rospy
    std_msgs
    urdf
)

add_library(FlexivRdk SHARED IMPORTED)
if (${BUILD_FOR_ARM64})
  set_target_properties(FlexivRdk PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/rdk/lib/libFlexivRdk.arm64-darwin.a)
else()
  set_target_properties(FlexivRdk PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/rdk/lib/libFlexivRdk.x86_64-linux-gnu.a)
endif()


add_executable(flexiv_task_ready_position
  src/flexiv_task_ready_position.cpp
)
target_include_directories(
  flexiv_task_ready_position
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/rdk/include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(flexiv_task_ready_position
  PRIVATE
  FlexivRdk
  ${catkin_LIBRARIES}
)

# INSTALL
# Install executable
install(
  TARGETS
    flexiv_task_ready_position
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)