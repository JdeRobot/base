cmake_minimum_required(VERSION 2.8)

SET( SOURCE_FILES ../src/ardrone_driver.cpp 
                  ../src/video.cpp 
		  ../src/ardrone_sdk.cpp 
		  ../src/teleop_twist.cpp
		  ../src/components/cameraserver.cpp 
		  ../src/components/cmdveli.cpp			
		  ../src/components/pose3di.cpp
		  ../src/components/remoteconfigi.cpp
		  ../src/components/navdatai.cpp			
		  ../src/components/ardroneextrai.cpp									
		  ../src/ardrone_parser.cpp
		  ../src/ardrone_config.cpp)

add_executable (ardrone_server ${SOURCE_FILES})

SET( LIBS_DIR /usr/local/lib/jderobot/)
SET( INCLUDES_JDEROBOT /usr/local/include/jderobot)
SET( LIBS_DIR_ARDRONE ${LIBS_DIR}/ardrone/)
SET( INCLUDES_PARROT  ${INCLUDES_JDEROBOT}/ardrone/)

SET( CMAKE_CXX_FLAGS "-lIce -lIceUtil" ) 

include(FindPkgConfig)
PKG_CHECK_MODULES(libxmlpp REQUIRED libxml++-2.6)
find_package(OpenCV REQUIRED)


FIND_PATH( Ice_INCLUDE_DIR NAMES Ice/Ice.h  PATHS ENV C++LIB ENV)
 
IF( Ice_INCLUDE_DIR )
    FIND_LIBRARY( Ice_LIBRARY1 NAMES Ice PATHS ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64 )
    FIND_LIBRARY( Ice_LIBRARY2 NAMES IceUtil PATHS ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64)
    SET (Ice_LIBRARIES ${Ice_LIBRARY1} ${Ice_LIBRARY2})
    IF( Ice_LIBRARIES )
		MESSAGE ("-- Ice found at ${Ice_LIBRARIES}")
		include_directories(${Ice_INCLUDE_DIR})
		link_directories(${Ice_LIBRARIES})
    ENDIF( Ice_LIBRARIES )
ENDIF(Ice_INCLUDE_DIR)

IF(NOT Ice_LIBRARIES)
		MESSAGE ("*** Ice not found")
ENDIF()

include_directories(
	  ${INCLUDES_PARROT}
	  ${INCLUDES_PARROT}/FFMPEG/Includes
	  ${INCLUDES_PARROT}/Soft/Common
	  ${INCLUDES_PARROT}/Soft/Lib
	  ${INCLUDES_PARROT}/VP_SDK
	  ${INCLUDES_PARROT}/VP_SDK/VP_Os/linux	
	  ${INCLUDES_JDEROBOT}
	  ${Boost_INCLUDE_DIR}
	  ${OpenCV_INCLUDE_DIR}
          ${libxmlpp_INCLUDE_DIRS} 	  
)

target_link_libraries(ardrone_server ${LIBS_DIR_ARDRONE}/libpc_ardrone.a
				     ${LIBS_DIR_ARDRONE}/libavcodec.a
				     ${LIBS_DIR_ARDRONE}/libavutil.a
				     ${LIBS_DIR_ARDRONE}/libswscale.a
				     ${LIBS_DIR_ARDRONE}/libvlib.a
				     ${LIBS_DIR_ARDRONE}/libsdk.a
				     SDL
				     ${CMAKE_THREAD_LIBS_INIT}
				     ${OpenCV_LIBS}
				     ${Ice_LIBRARIES}
				     ${LIBS_DIR}/libcolorspacesmm.so
				     ${LIBS_DIR}/libJderobotInterfaces.so
				     ${libxmlpp_LIBRARIES}
)
