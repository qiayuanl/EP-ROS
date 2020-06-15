cmake_minimum_required(VERSION 2.8.3)
project(ep_base)

find_package(catkin REQUIRED COMPONENTS
    rospy
    )

catkin_package(
)

include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS base.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

add_subdirectory(robomaster/module/decoder/src)