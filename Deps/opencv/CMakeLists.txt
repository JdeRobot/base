FIND_PATH( OpenCVHG_INCLUDE_DIR NAMES highgui.h PATHS ${OpenCV_DIR} ENV C++LIB ENV PATH PATH_SUFFIXES include/opencv/ )
message("dir: ${OpenCV_DIR}")

IF (OpenCVHG_INCLUDE_DIR)
 
	MESSAGE("OpenCVHighGui was found in: ${OpenCVHG_INCLUDE_DIR}")
  
	FIND_PATH( OpenCV_INCLUDE_DIR NAMES include/opencv/cv.h PATHS ${OpenCV_DIR}  ENV C++LIB ENV PATH  )
	INCLUDE_DIRECTORIES(
			${OpenCV_INCLUDE_DIR}/include/
			${OpenCV_INCLUDE_DIR}/include/opencv
			${OpenCV_INCLUDE_DIR}/include/opencv2
	)
	MESSAGE("inc dir: ${OpenCV_INCLUDE_DIR}")
	IF( OpenCV_INCLUDE_DIR )
    	MESSAGE("OpenCV was found in: ${OpenCV_INCLUDE_DIR}")
	
	    FIND_LIBRARY( OpenCV_LIBRARY1 NAMES opencv_core PATHS ${OpenCV_DIR} ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64)
	    FIND_LIBRARY( OpenCV_LIBRARY2 NAMES opencv_highgui PATHS ${OpenCV_DIR} ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64)
	    FIND_LIBRARY( OpenCV_LIBRARY3 NAMES opencv_contrib PATHS ${OpenCV_DIR} ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64)
	    FIND_LIBRARY( OpenCV_LIBRARY4 NAMES opencv_calib3d PATHS ${OpenCV_DIR} ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64)
	    FIND_LIBRARY( OpenCV_LIBRARY5 NAMES opencv_flann PATHS ${OpenCV_DIR} ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64)
	    FIND_LIBRARY( OpenCV_LIBRARY6 NAMES opencv_gpu PATHS ${OpenCV_DIR} ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64)
	    FIND_LIBRARY( OpenCV_LIBRARY7 NAMES opencv_imgproc PATHS ${OpenCV_DIR} ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64)
	    FIND_LIBRARY( OpenCV_LIBRARY8 NAMES opencv_legacy PATHS ${OpenCV_DIR} ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64)
	    FIND_LIBRARY( OpenCV_LIBRARY9 NAMES opencv_ml PATHS ${OpenCV_DIR} ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64)
	    FIND_LIBRARY( OpenCV_LIBRARY10 NAMES opencv_features2d PATHS ${OpenCV_DIR} ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64)
	    FIND_LIBRARY( OpenCV_LIBRARY11 NAMES opencv_objdetect PATHS ${OpenCV_DIR} ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64)
	    FIND_LIBRARY( OpenCV_LIBRARY14 NAMES opencv_video PATHS ${OpenCV_DIR} ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64)
	    SET (OpenCV_LIBRARIES ${OpenCV_LIBRARY1} ${OpenCV_LIBRARY5}  ${OpenCV_LIBRARY7}  ${OpenCV_LIBRARY9} ${OpenCV_LIBRARY10}  ${OpenCV_LIBRARY14} )
	    SET (OpenCVGUI_LIBRARIES ${OpenCV_LIBRARY2} ${OpenCV_LIBRARY3} ${OpenCV_LIBRARY4} ${OpenCV_LIBRARY6} ${OpenCV_LIBRARY8} ${OpenCV_LIBRARY11})

	    IF( OpenCV_LIBRARIES )
			#MESSAGE("OPENCV LIBRARY FOUND IN ${OpenCV_LIBRARY2}")

			link_directories(${OpenCV_LIBRARIES})	
		    include_directories(${OpenCV_INCLUDE_DIR})

			SET(DEPS "${DEPS}libopencv-calib3d2.4, libopencv-contrib2.4, libopencv-core2.4, libopencv-features2d2.4, libopencv-flann2.4, libopencv-gpu2.4, libopencv-highgui2.4, libopencv-imgproc2.4, libopencv-legacy2.4, libopencv-ml2.4, libopencv-objdetect2.4, libopencv-video2.4, libcv2.4, libcvaux2.4, libhighgui2.4,")

	    ENDIF( OpenCV_LIBRARIES )


	ELSE(OpenCV_INCLUDE_DIR)
		MESSAGE( FATAL_ERROR "*** OpenCV not found")
	ENDIF(OpenCV_INCLUDE_DIR)

ELSE(OpenCVHG_INCLUDE_DIR)

	MESSAGE(FATAL_ERROR "*** OpenCV: libhighgui-dev not found!")

ENDIF(OpenCVHG_INCLUDE_DIR)
