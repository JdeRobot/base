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



#TRY MANUAL IF AUTO NOT WORKS
IF (NOT DEFINED OpenCV_VERSION)
	IF (NOT OpenCV_DIR)
		SET(OpenCV_DIR /opt/ros/kinetic/)
	ENDIF()
	FIND_PATH( OpenCV_INCLUDE_DIR NAMES highgui.h PATHS ${OpenCV_DIR} ENV C++LIB ENV PATH PATH_SUFFIXES include/opencv/ )
	IF (NOT OpenCV_INCLUDE_DIR)
		#TRY ROS OPENCV PATH
		FIND_PATH( OpenCV_INCLUDE_DIR NAMES opencv/highgui.h PATHS ${OpenCV_DIR} ENV C++LIB ENV PATH PATH_SUFFIXES include/opencv-3.2.0-dev)
		SET(OPENCV_ROS ON)
	ENDIF()
	IF (OpenCV_INCLUDE_DIR)
		MESSAGE("OpenCV was found in: ${OpenCV_DIR}")

		IF (OPENCV_ROS)
			INCLUDE_DIRECTORIES(
					${OpenCV_INCLUDE_DIR}/
					${OpenCV_INCLUDE_DIR}/opencv
					${OpenCV_INCLUDE_DIR}/opencv2
			)
			SET(OPENCV_MODULE_SUFIX "3")
		ELSE()
			INCLUDE_DIRECTORIES(
					${OpenCV_INCLUDE_DIR}/include/
					${OpenCV_INCLUDE_DIR}/include/opencv
					${OpenCV_INCLUDE_DIR}/include/opencv2
			)
			SET(OPENCV_MODULE_SUFIX "")
		ENDIF()

		SET(OPENCV_MODULES opencv_core opencv_flann opencv_imgproc opencv_ml opencv_features2d opencv_video)
		SET(OPENCV_GUI_MODULES opencv_highgui opencv_calib3d opencv_objdetect)

		FOREACH (MOD ${OPENCV_MODULES})
			MESSAGE("LOOKING FOR ${MOD}${OPENCV_MODULE_SUFIX} INTO ${OpenCV_DIR}")
	    		FIND_LIBRARY( OpenCV_LIBRARY_${MOD} NAMES ${MOD}${OPENCV_MODULE_SUFIX} PATHS ${OpenCV_DIR} ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64)
			IF ( NOT OpenCV_LIBRARY_${MOD})
				MESSAGE(FATAL_ERROR "OPENCV MODULE: ${MOD} NOT FOUND")
			ELSE()
				SET (OpenCV_LIBRARIES ${OpenCV_LIBRARIES} ${OpenCV_LIBRARY_${MOD}})
			ENDIF()
		ENDFOREACH()

		FOREACH (MOD ${OPENCV_GUI_MODULES})
	    		FIND_LIBRARY( OpenCV_LIBRARY_${MOD} NAMES ${MOD}${OPENCV_MODULE_SUFIX} PATHS ${OpenCV_DIR} ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64)
			IF ( NOT OpenCV_LIBRARY_${MOD})
				MESSAGE(FATAL_ERROR "OPENCV MODULE: ${MOD} NOT FOUND")
			ELSE()
				SET (OpenCVGUI_LIBRARIES ${OpenCVGUI_LIBRARIES} ${OpenCV_LIBRARY_${MOD}})
			ENDIF()
		ENDFOREACH()


	ELSE(OpenCV_INCLUDE_DIR)
		MESSAGE(FATAL_ERROR "*** OpenCV: not found!")
	ENDIF(OpenCV_INCLUDE_DIR)

ENDIF()

message("***OpenCV Version: ${OpenCV_VERSION}")
message("***OpenCV CORE LIBS: ${OpenCV_LIBRARIES}")
message("***OpenCV GUI LIBS: ${OpenCVGUI_LIBRARIES}")
message("***OpenCV EXTRA LIBS: ${OpenCV_EXTRA_LIBS}")
