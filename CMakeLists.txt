cmake_minimum_required(VERSION 2.8.3)
project(dataset_ros)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

set(PACKAGE_DEPS 
  roscpp rospy std_msgs
  geometry_msgs sensor_msgs
  nodelet
  cartographer_ros manual_measurements)

set(PYTHON_SCRIPTS
  scripts/extract_trajectory.py)

find_package(cartographer REQUIRED)
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
google_initialize_cartographer_project()


find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES err_check_nodelet
  CATKIN_DEPENDS ${PACKAGE_DEPS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

add_library(err_check_nodelet src/err_check/nodelet.cc)
target_link_libraries(err_check_nodelet ${catkin_LIBRARIES})

add_executable(err_check src/err_check/node.cc)
target_link_libraries(err_check ${catkin_LIBRARIES})

add_executable(dump_trajectory src/dump_trajectory.cc)
target_link_libraries(dump_trajectory ${catkin_LIBRARIES})
add_dependencies(dump_trajectory ${catkin_EXPORTED_TARGETS})
set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(dump_trajectory PROPERTIES 
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})


install(TARGETS err_check err_check_nodelet dump_trajectory
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


install(DIRECTORY launch configuration_files
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#catkin_install_python(PROGRAMS ${PYTHON_SCRIPTS}
  #DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

