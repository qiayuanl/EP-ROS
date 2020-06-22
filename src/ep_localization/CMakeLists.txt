cmake_minimum_required(VERSION 2.8.3)
project(ep_localization)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    )

include_directories(
    ./include
    ${catkin_INCLUDE_DIRS}
)

#tags_lut
add_executable(tags_lut src/tags_lut/tags_lut.cpp)
target_link_libraries(tags_lut ${catkin_LIBRARIES})

#tags_localization
add_executable(tags_localization src/tags_localizaiton/tags_local_node.cpp src/tags_localizaiton/tags_localization.cpp)
target_link_libraries(tags_localization ${catkin_LIBRARIES})