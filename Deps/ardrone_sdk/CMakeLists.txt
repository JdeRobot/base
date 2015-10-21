cmake_minimum_required(VERSION 2.8)

SET(ARDRONE_ALL_DEPS true)

FIND_PATH( ARDRONE_SDK NAMES ardrone  PATHS ENV C++LIB ENV PATH PATH_SUFFIXES jderobot)

IF(ARDRONE_SDK)
	SET(ARDRONE_LIB_PATH "/usr/local/lib/jderobot/ardrone")
	message(STATUS "ardrone_sdk found at ${ARDRONE_SDK}")
	include_directories(${ARDRONE_SDK}/ardrone 
			    ${ARDRONE_SDK}/ardrone/FFMPEG/Includes 
                            ${ARDRONE_SDK}/ardrone/Soft/Common 
                            ${ARDRONE_SDK}/ardrone/Soft/Lib 
                            ${ARDRONE_SDK}/ardrone/VP_SDK 
                            ${ARDRONE_SDK}/ardrone/VP_SDK/VP_Os/linux )

	FIND_LIBRARY( ARDRONE_AVDEVICE_LIB NAMES avdevice PATHS ${ARDRONE_LIB_PATH} NO_DEFAULT_PATH)
	IF( ARDRONE_AVDEVICE_LIB )
		message(STATUS "libavdevice found at ${ARDRONE_AVDEVICE_LIB}")
	ELSE()
		message("libavdevice library not found")
		SET(ARDRONE_ALL_DEPS false)
	ENDIF()

	FIND_LIBRARY( ARDRONE_AVCODEC_LIB NAMES avcodec PATHS ${ARDRONE_LIB_PATH} NO_DEFAULT_PATH)
	IF( ARDRONE_AVCODEC_LIB )
		message(STATUS "libavcodec found at ${ARDRONE_AVCODEC_LIB}")
	ELSE()
		message("libavcodec library not found")
		SET(ARDRONE_ALL_DEPS false)
	ENDIF()

	FIND_LIBRARY( ARDRONE_AVFILTER_LIB NAMES avfilter PATHS ${ARDRONE_LIB_PATH} NO_DEFAULT_PATH)
	IF( ARDRONE_AVFILTER_LIB )
		message(STATUS "libavfilter found at ${ARDRONE_AVFILTER_LIB}")
	ELSE()
		message("libavfilter library not found")
		SET(ARDRONE_ALL_DEPS false)
	ENDIF()

	FIND_LIBRARY( ARDRONE_AVFORMAT_LIB NAMES avformat PATHS ${ARDRONE_LIB_PATH} NO_DEFAULT_PATH)
	IF( ARDRONE_AVFORMAT_LIB )
		message(STATUS "libavformat found at ${ARDRONE_AVFORMAT_LIB}")
	ELSE()
		message("libavformat library not found")
		SET(ARDRONE_ALL_DEPS false)	
	ENDIF()

	FIND_LIBRARY( ARDRONE_AVUTIL_LIB NAMES avutil PATHS ${ARDRONE_LIB_PATH} NO_DEFAULT_PATH)
	IF( ARDRONE_AVUTIL_LIB )
		message(STATUS "libavutil found at ${ARDRONE_AVUTIL_LIB}")
	ELSE()
		message("libavutil library not found")
		SET(ARDRONE_ALL_DEPS false)
	ENDIF()

	FIND_LIBRARY( ARDRONE_SWSCALE_LIB NAMES swscale PATHS ${ARDRONE_LIB_PATH} NO_DEFAULT_PATH)
	IF( ARDRONE_SWSCALE_LIB )
		message(STATUS "libswscale found at ${ARDRONE_SWSCALE_LIB}")
	ELSE()
		message("libswscale library not found")
		SET(ARDRONE_ALL_DEPS false)
	ENDIF()

	FIND_LIBRARY( ARDRONE_PCDRONE_LIB NAMES pc_ardrone PATHS ${ARDRONE_LIB_PATH} NO_DEFAULT_PATH)
	IF( ARDRONE_PCDRONE_LIB )
		message(STATUS "libpc_ardrone found at ${ARDRONE_PCDRONE_LIB}")
	ELSE()
		message("libpc_ardrone library not found")
		SET(ARDRONE_ALL_DEPS false)
	ENDIF()

#	FIND_LIBRARY( ARDRONE_PCDRONENOTOOL_LIB NAMES pc_ardrone_notool PATHS ${ARDRONE_ARDRONE_LIB_PATH} NO_DEFAULT_PATH)
#	IF( ARDRONE_PCDRONENOTOOL_LIB )
#		message(STATUS "libpc_ardrone_notool found at ${ARDRONE_PCDRONENOTOOL_LIB}")
#	ELSE()
#		message("libpc_ardrone_notool library not found")
#		SET(ARDRONE_ALL_DEPS false)
#	ENDIF()

	FIND_LIBRARY( ARDRONE_SDK_LIB NAMES sdk PATHS ${ARDRONE_LIB_PATH} NO_DEFAULT_PATH)
	IF( ARDRONE_SDK_LIB )
		message(STATUS "libsdk found at ${ARDRONE_SDK_LIB}")
	ELSE()
		message("libsdk library not found")
		SET(ARDRONE_ALL_DEPS false)
	ENDIF()

	FIND_LIBRARY( ARDRONE_VLIB_LIB NAMES vlib PATHS ${ARDRONE_LIB_PATH} NO_DEFAULT_PATH)
	IF( ARDRONE_VLIB_LIB )
		message(STATUS "libvlib found at ${ARDRONE_VLIB_LIB}")
	ELSE()
		message("libvlib library not found")
		SET(ARDRONE_ALL_DEPS false)
	ENDIF()

ELSE()
	SET(ARDRONE_ALL_DEPS false)
ENDIF()
