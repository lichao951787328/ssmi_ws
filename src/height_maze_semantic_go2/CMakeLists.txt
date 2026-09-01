cmake_minimum_required(VERSION 3.0.2)
project(height_maze_semantic_go2)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/generate_fortress_world.py
    scripts/generate_dynamic_fortress_world.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
