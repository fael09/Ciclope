cmake_minimum_required(VERSION 2.8)
project( teste )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( teste teste.cpp )
target_link_libraries( teste ${OpenCV_LIBS} )