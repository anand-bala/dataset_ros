cmake_minimum_required(VERSION 2.8.3)
project(dataset_ros)

find_package(catkin REQUIRED
  roscpp rospy std_msgs
  genmsg actionlib_msgs actionlib
)


catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs
)

SET(SCRIPTS
  nodes/moveforward.py
)

install(DIRECTORY launch configuration_files
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS ${SCRIPTS}
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

