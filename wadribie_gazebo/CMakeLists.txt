cmake_minimum_required(VERSION 2.8.3)
project(wadribie_gazebo)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY worlds/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)
