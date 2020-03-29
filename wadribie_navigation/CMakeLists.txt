cmake_minimum_required(VERSION 2.8.3)
project(wadribie_navigation)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(PROGRAMS scripts/navigation.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
