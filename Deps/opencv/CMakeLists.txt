SET (opencv_core_libs_components core flann imgproc ml features2d video objdetect )
SET (opencv_gui_libs_components highgui calib3d)
SET (opencv2_extra_libs_components contrib gpu legacy)
SET (opencv3_extra_libs_components bgsegm )

FIND_PACKAGE(OpenCV QUIET)

#look for core libs
UNSET(OpenCV_LIBS)
FIND_PACKAGE(OpenCV COMPONENTS ${opencv_core_libs_components})
SET(OpenCV_CORE_LIBS ${OpenCV_LIBS})

#look for gui libs
UNSET(OpenCV_LIBS)
FIND_PACKAGE(OpenCV COMPONENTS ${opencv_gui_libs_components})
SET(OpenCV_GUI_LIBS ${OpenCV_LIBS})

IF (DEFINED OpenCV_VERSION)
	#foreach( d ${opencv_core_libs_components})
	#	list(APPEND DEPS "libopencv-${d}${OpenCV_VERSION_MAJOR}.${OpenCV_VERSION_MINOR}")
	#	list(APPEND DEPS_DEV "libopencv-${d}-dev")
	#endforeach()

	IF (${OpenCV_VERSION_MAJOR} MATCHES "3")
		add_definitions(-D_OPENCV3)
		#look for extra libs 
		UNSET(OpenCV_LIBS)
		FIND_PACKAGE(OpenCV COMPONENTS ${opencv3_extra_libs_components})
		SET(OpenCV_EXTRA_LIBS ${OpenCV_LIBS})
		#foreach( d ${opencv3_extra_libs_components})
		#	list(APPEND DEPS "libopencv-${d}${OpenCV_VERSION_MAJOR}.${OpenCV_VERSION_MINOR}")
		#	list(APPEND DEPS_DEV "libopencv-${d}-dev")
		#endforeach()
	ELSE()
		#look for extra libs 
		UNSET(OpenCV_LIBS)
		FIND_PACKAGE(OpenCV COMPONENTS ${opencv2_extra_libs_components})
		SET(OpenCV_EXTRA_LIBS ${OpenCV_LIBS})
		#foreach( d ${opencv2_extra_libs_components})
		#	list(APPEND DEPS "libopencv-${d}${OpenCV_VERSION_MAJOR}.${OpenCV_VERSION_MINOR}")
		#	list(APPEND DEPS_DEV "libopencv-${d}-dev")
		#endforeach()

	ENDIF()
	list(APPEND DEPS "ros-kinetic-opencv3")
	list(APPEND DEPS_DEV "ros-kinetic-opencv3")
ENDIF()




include_directories(${OpenCV_INCLUDE_DIRS})
#SET TO JDEROBOT CMAKE VARIABLES
SET(OpenCV_LIBRARIES ${OpenCV_CORE_LIBS})
SET(OpenCVGUI_LIBRARIES ${OpenCV_GUI_LIBS})


message("***OpenCV Version: ${OpenCV_VERSION}")
message("***OpenCV CORE LIBS: ${OpenCV_CORE_LIBS}")
message("***OpenCV GUI LIBS: ${OpenCV_GUI_LIBS}")
message("***OpenCV EXTRA LIBS: ${OpenCV_EXTRA_LIBS}")
