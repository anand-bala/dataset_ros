cmake_minimum_required(VERSION 2.8.3)
project(dataset_ros)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs
  genmsg actionlib_msgs actionlib
  nodelet
  roslint
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES err_check_nodelet
  CATKIN_DEPENDS 
    roscpp rospy std_msgs
    geometry_msgs sensor_msgs
    nodelet
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(err_check_nodelet src/err_check/nodelet.cc)
target_link_libraries(err_check_nodelet ${catkin_LIBRARIES})

add_executable(err_check src/err_check/node.cc)
target_link_libraries(err_check ${catkin_LIBRARIES})

roslint_cpp()

install(TARGETS err_check err_check_nodelet
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


install(DIRECTORY launch configuration_files
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

