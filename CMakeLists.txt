cmake_minimum_required(VERSION 2.8.3)
project(dataset_ros)

find_package(catkin REQUIRED roslaunch)

catkin_package(
    
)

install(DIRECTORY launch configuration_files
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS ${SCRIPTS_SRC}
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

