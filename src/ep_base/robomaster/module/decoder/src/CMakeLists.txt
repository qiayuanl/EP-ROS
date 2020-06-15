cmake_minimum_required(VERSION 3.4.1)
project(robomaster_video_and_audio_decoder)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Opus REQUIRED)
#find_package(pybind11 REQUIRED)
add_subdirectory(pybind11)

set(PYBIND11_PYTHON_VERSION 2.7)
set(PYBIND11_CPP_STANDARD -std=c++11)

pybind11_add_module(opus_decoder opus_decoder/opus_decoder.cpp)
target_link_libraries(opus_decoder PRIVATE ${OPUS_LIBRARIES})
target_include_directories(opus_decoder PRIVATE ${OPUS_INCLUDE_DIRS})

pybind11_add_module(libh264decoder h264_decoder/h264decoder.cpp)
target_link_libraries(libh264decoder PRIVATE avcodec swscale avutil)

set_target_properties(opus_decoder PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../)

set_target_properties(libh264decoder PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../)