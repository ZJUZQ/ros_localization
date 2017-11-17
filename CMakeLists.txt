cmake_minimum_required(VERSION 2.8)
project( ros_localization )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

include_directories( /home/vortex/3rdparty/include )
#link_libraries( /home/vortex/zhou_temp_test/visual_tracker/lib )

add_executable( visual_localization src/visual_localization.cpp )
target_link_libraries( visual_localization ${OpenCV_LIBS} /home/vortex/3rdparty/lib/libBOOSTING.so)