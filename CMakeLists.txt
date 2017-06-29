cmake_minimum_required(VERSION 2.8.3)
project(vrpn_client_ros)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp roslaunch roslint tf2_ros)
find_package(VRPN REQUIRED)
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
  message(WARN "Using Eigen2!")
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vrpn_client_ros
  CATKIN_DEPENDS geometry_msgs tf2_ros
  DEPENDS VRPN
)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall")

include_directories(
  include
  include/vrpn_client_ros
  ${catkin_INCLUDE_DIRS}
  ${VRPN_INCLUDE_DIR}
)

add_library(vrpn_client_ros
  src/vrpn_client_ros.cpp
)
target_link_libraries(vrpn_client_ros
  ${catkin_LIBRARIES}
  ${VRPN_LIBRARIES}
)

add_executable(vrpn_tracker_node
  src/vrpn_tracker_node.cpp src/LowPassFilter.cpp src/filtered_derivative_wcb.cpp)
add_dependencies(vrpn_tracker_node vrpn_client_ros)
target_link_libraries(vrpn_tracker_node
  vrpn_client_ros
  ${catkin_LIBRARIES}
)

add_executable(vrpn_client_node
  src/vrpn_client_node.cpp src/LowPassFilter.cpp src/filtered_derivative_wcb.cpp)
add_dependencies(vrpn_client_node vrpn_client_ros)
target_link_libraries(vrpn_client_node
  vrpn_client_ros
  ${catkin_LIBRARIES}
)

roslaunch_add_file_check(launch)

file(GLOB_RECURSE SOURCE_FILES src *.cpp)
file(GLOB_RECURSE HEADER_FILES include *.h)

roslint_cpp(
  ${SOURCE_FILES}
  ${HEADER_FILES}
  )

install(TARGETS vrpn_client_ros vrpn_tracker_node vrpn_client_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
