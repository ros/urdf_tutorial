cmake_minimum_required(VERSION 2.8.3)
project(urdf_tutorial)

find_package(catkin REQUIRED roslaunch)
catkin_package()

if(CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch)
endif()

install(DIRECTORY config images meshes launch rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
