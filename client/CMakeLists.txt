set( TARGET coyote_racer_client )

cmake_minimum_required( VERSION 2.8 )

find_package( OpenCV 3 REQUIRED )

set( SOCKET_SRC_DIR ../common/src )

include_directories( src ${SOCKET_SRC_DIR} )
file( GLOB SRCS src/*.cpp )

# to enable debug mode (!!Note: Clean build directory before changing mode)
#  > cmake -Ddebug=1 ..
if( debug )
  set( CMAKE_BUILD_TYPE Debug )
else()
  set( CMAKE_BUILD_TYPE Release )
endif()

set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -Ofast -DNDEBUG -march=native" )
set( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11 -O0 -g3" )

add_executable( ${TARGET} ${SRCS} ${SOCKET_SRC_DIR}/comm_socket.cpp )
target_link_libraries( ${TARGET} opencv_core opencv_imgproc opencv_highgui pthread )
