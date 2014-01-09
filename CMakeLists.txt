cmake_minimum_required(VERSION 2.8.3)
project(urdf_tutorial)

find_package(catkin)
catkin_package()

foreach(dir images launch urdf)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(
  FILES urdf.rviz urdf.vcg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
