cmake_minimum_required(VERSION 3.0.2)
project(semantic_segmentation_go2)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
  unitree_legged_msgs
)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

catkin_package()

add_library(go2_effort_pd_system SHARED
  src/go2_effort_pd_system.cpp
)
target_link_libraries(go2_effort_pd_system
  ignition-gazebo6::ignition-gazebo6
  ignition-transport11::ignition-transport11
  ignition-msgs8::ignition-msgs8
)

catkin_install_python(PROGRAMS
  scripts/dynamic_tf_go2.py
  scripts/dynamic_semantic_remap.py
  scripts/go2_fortress_control_adapter.py
  scripts/sdf_sensor_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch models rviz sdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS go2_effort_pd_system
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
