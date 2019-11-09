set( TARGET coyote_racer_server )
set( RASPIVIDEOCAP_DIR /opt/raspivideocap )

set( SOCKET_SRC_DIR ../common/src )

cmake_minimum_required( VERSION 2.8 )

find_package( OpenCV 3 REQUIRED )

include_directories( ${RASPIVIDEOCAP_DIR}/include src ${SOCKET_SRC_DIR} )
link_directories( /opt/vc/lib ${RASPIVIDEOCAP_DIR}/lib )

file( GLOB SRCS src/*.cpp )

# to enable debug mode (!!Note: Clean build directory before changing mode)
#  > cmake -Ddebug=1 ..
if( debug )
  set( CMAKE_BUILD_TYPE Debug )
else()
  set( CMAKE_BUILD_TYPE Release )
endif()

set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG -march=native" )
set( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g3" )

set( MMAL_LIBS mmal_core mmal mmal_util vcos bcm_host )
set( OPENCV_LIBS opencv_core opencv_imgproc opencv_aruco opencv_calib3d opencv_ccalib opencv_highgui )

add_executable( ${TARGET} ${SRCS} ${SOCKET_SRC_DIR}/comm_socket.cpp )
target_link_libraries( ${TARGET} ${MMAL_LIBS} raspivideocap ${OPENCV_LIBS} pthread rt pigpio )
