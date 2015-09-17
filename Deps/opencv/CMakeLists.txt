SET (opencv_core_libs_components core flann imgproc ml features2d video objdetect )
SET (opencv_gui_libs_components highgui calib3d)
SET (opencv2_extra_libs_components contrib gpu legacy)
SET (opencv3_extra_libs_components )

FIND_PACKAGE(OpenCV REQUIRED)
message("verison ${OpenCV_VERSION_MAJOR}")

#look for core libs
UNSET(OpenCV_LIBS)
FIND_PACKAGE(OpenCV COMPONENTS ${opencv_core_libs_components})
SET(OpenCV_CORE_LIBS ${OpenCV_LIBS})

#look for gui libs
UNSET(OpenCV_LIBS)
FIND_PACKAGE(OpenCV COMPONENTS ${opencv_gui_libs_components})
SET(OpenCV_GUI_LIBS ${OpenCV_LIBS})

IF (${OpenCV_VERSION_MAJOR} MATCHES "3")
ELSE()
	#look for extra libs 
	UNSET(OpenCV_LIBS)
	FIND_PACKAGE(OpenCV COMPONENTS ${opencv2_extra_libs_components})
	SET(OpenCV_EXTRA_LIBS ${OpenCV_LIBS})
ENDIF()


message("***OpenCV Version: ${OpenCV_VERSION}")
message("***OpenCV CORE LIBS: ${OpenCV_CORE_LIBS}")
message("***OpenCV GUI LIBS: ${OpenCV_GUI_LIBS}")
message("***OpenCV EXTRA LIBS: ${OpenCV_EXTRA_LIBS}")


#SET TO JDEROBOT CMAKE VARIABLES
SET(OpenCV_LIBRARIES ${OpenCV_CORE_LIBS})
SET(OpenCVGUI_LIBRARIES ${OpenCV_GUI_LIBS})
