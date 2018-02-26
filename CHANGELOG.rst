^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package urdf_tutorial
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.5 (2017-04-28)
------------------
* New ROS Control Tutorial
* Remove dependency on pr2_description (uses local meshes instead)
* General cleanup, bug fixes and reorganization
* Contributors: David V. Lu!!, Felix Duvallet, Paul Bovbel, Thibault Kruse

0.2.4 (2015-04-24)
------------------
* Maintainer list changes
* Add .rviz as an arg
* fix gazebo.launch for indigo, update to new package name
* Add run depends
* Contributors: David V. Lu, Ioan A Sucan, Isaac IY Saito, Kei Okada

0.2.3 (2014-01-09)
------------------
* Merge pull request `#7 <https://github.com/ros/urdf_tutorial/issues/7>`_ from gavanderhoorn/issue6_source_vs_devel_vs_install
  Fix issue `#6 <https://github.com/ros/urdf_tutorial/issues/6>`_: No robot model when using display.launch (again)
* Setup new directory structure and move files around.
* Contributors: Ioan A Sucan, gavanderhoorn

0.2.2 (2013-12-16)
------------------
* Merge pull request `#5 <https://github.com/ros/urdf_tutorial/issues/5>`_ from gavanderhoorn/issue4_missing_robot_model_display
  Fix issue `#4 <https://github.com/ros/urdf_tutorial/issues/4>`_: missing RobotModel display
* Make roslaunch stop all nodes on RViz exit.
* Update path to RViz config in display.launch. Fix `#4 <https://github.com/ros/urdf_tutorial/issues/4>`_.
  Seems xacrodisplay.launch was already fixed, but somehow display.launch was not updated. See `#3 <https://github.com/ros/urdf_tutorial/issues/3>`_.
* fix `#3 <https://github.com/ros/urdf_tutorial/issues/3>`_
* Contributors: Ioan Sucan, gavanderhoorn

0.2.1 (2013-05-28 14:04)
------------------------
* update package name and install location for compatibility with previous build system
* Contributors: Ioan Sucan

0.2.0 (2013-05-28 13:47)
------------------------
* catkinizing package
* Merge pull request `#1 <https://github.com/ros/urdf_tutorial/issues/1>`_ from ros-industrial/master
  Rviz config file update
* Ported rviz config file to new groovy format.  Updated launch files to use new config file
* Added tag unstable for changeset bd8da58ba34e
* Added tag robot_model_tutorials-0.1.2 for changeset 6ac373f839e0
* getting ready for 0.1.2 release
* Added tag unstable for changeset dcec5b1f8ed8
* Added tag robot_model_tutorials-0.1.1 for changeset e023b0707337
* getting ready for 0.1.1 release
* Automated merge with https://kforge.ros.org/robotmodel/tutorials
* update stack dependencies
* Added tag unstable for changeset b22bbf7d7850
* Added tag robot_model_tutorials-0.1.0 for changeset a0b3b46efc95
* initial copy
* Updated meta-data, JointStatePublisher with merging ability
* Cleaning up some manifests
* URDF Tutorial
* Bug Fix 3101648
* Joint Controller renamed and reworked as Joint_state_publisher
* Screenshot for Joint controller
* Initial URDF Tools Package
* Contributors: Ioan Sucan, Shaun Edwards, Wim Meeussen, isucan, probablydavid
