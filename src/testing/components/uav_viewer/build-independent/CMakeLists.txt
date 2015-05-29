cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(uav_viewer)
include (FindPkgConfig)


set(GUI_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/gui)

MACRO (QT4_WRAP_UI_CUSTOM outfiles )
  QT4_EXTRACT_OPTIONS(ui_files ui_options ${ARGN})

  FOREACH (it ${ui_files})
    GET_FILENAME_COMPONENT(outfile ${it} NAME_WE)
    GET_FILENAME_COMPONENT(infile ${it} ABSOLUTE)
    SET(outfile ${GUI_OUTPUT_PATH}/ui_${outfile}.h)
    ADD_CUSTOM_COMMAND(OUTPUT ${outfile}
      COMMAND ${QT_UIC_EXECUTABLE}
      ARGS ${ui_options} -o ${outfile} ${infile}
      MAIN_DEPENDENCY ${infile})
    SET(${outfiles} ${${outfiles}} ${outfile})
  ENDFOREACH (it)

ENDMACRO (QT4_WRAP_UI_CUSTOM)

find_package(Qt4 REQUIRED)

include(${QT_USE_FILE})

# Qwt include directory
set(QWT_INCLUDE_DIR /usr/include/qwt-qt4)
if(NOT QWT_INCLUDE_DIR)
  message(FATAL_ERROR "QWT_INCLUDE_DIR should be set to a path like ")
endif()
include_directories(${QWT_INCLUDE_DIR})

# Qwt libraries
set(QWT_LIBRARY /usr/lib/libqwt-qt4.so)
if(NOT QWT_LIBRARY)
  message(FATAL_ERROR "QWT_LIBRARY should be set to a filepath like ")
endif()

set(qwt_libraries ${QWT_LIBRARY})

SET(qt_SOURCES ../sensors/threadsensors.cpp ../gui/threadgui.cpp ../gui/mainwindow.cpp 
	       ../gui/attitudeindicator.cpp ../gui/speedometer.cpp
)

SET(qt_HEADERS	../sensors/threadsensors.h ../gui/threadgui.h ../gui/mainwindow.h 
		../gui/attitudeindicator.h ../gui/speedometer.h

)

SET(qt_FORMS ../gui/mainwindow.ui)

QT4_WRAP_CPP(qt_HEADERS_MOC ${qt_HEADERS})
QT4_WRAP_UI_CUSTOM(qt_FORMS_HEADERS ${qt_FORMS})
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

include(FindPkgConfig)
#manual opencv
FIND_PATH( OpenCV_INCLUDE_DIR NAMES cv.h  PATHS ENV C++LIB ENV PATH PATH_SUFFIXES include/opencv/)
IF( OpenCV_INCLUDE_DIR )
	MESSAGE("OpenCV was found in: ${OpenCV_INCLUDE_DIR}")
	
    FIND_LIBRARY( OpenCV_LIBRARY1 NAMES opencv_core PATHS ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64)
    FIND_LIBRARY( OpenCV_LIBRARY2 NAMES opencv_highgui PATHS ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64)
    FIND_LIBRARY( OpenCV_LIBRARY3 NAMES opencv_imgproc PATHS ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64)

    SET (OpenCV_LIBRARIES ${OpenCV_LIBRARY1})
    SET (OpenCVGUI_LIBRARIES ${OpenCV_LIBRARY2} ${OpenCV_LIBRARY3} )
    IF( OpenCV_LIBRARIES )
		#MESSAGE("OPENCV LIBRARY FOUND IN ${OpenCV_LIBRARIES}")
    ENDIF( OpenCV_LIBRARIES )
ENDIF(OpenCV_INCLUDE_DIR)

IF (NOT  OpenCV_INCLUDE_DIR)
	MESSAGE("*** OpenCV not found")
ENDIF()

#manual ICE
FIND_PATH( Ice_INCLUDE_DIR NAMES Ice/Ice.h  PATHS ENV C++LIB ENV)

IF( Ice_INCLUDE_DIR )
    FIND_LIBRARY( Ice_LIBRARY1 NAMES Ice PATHS ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64 )
    FIND_LIBRARY( Ice_LIBRARY2 NAMES IceUtil PATHS ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64)
    SET (Ice_LIBRARIES ${Ice_LIBRARY1} ${Ice_LIBRARY2})
    IF( Ice_LIBRARIES )
                MESSAGE ("-- Ice found at ${Ice_LIBRARIES}")
    ENDIF( Ice_LIBRARIES )
ENDIF(Ice_INCLUDE_DIR)

IF(NOT Ice_LIBRARIES)
                MESSAGE ("*** Ice not found")
ENDIF()


SET( LIBS_DIR /usr/local/lib/jderobot) 

include_directories(
	/usr/local/include/jderobot
	${OpenCV_INCLUDE_DIR}
        ${Ice_INCLUDE_DIR}
)
link_directories(${JDE_LIBS})

find_package(Threads)

add_executable(uav_viewer 
	../src/main.cpp
	../sensors/sensors.cpp	
	../resources/images.cpp
	${qt_SOURCES} 
	${qt_FORMS} 
        ${qt_HEADERS_MOC})

target_link_libraries(uav_viewer
        ${CMAKE_THREAD_LIBS_INIT}
	${QT_LIBRARIES}
	${qwt_libraries}
	${OpenCV_LIBRARIES}
        ${OpenCVGUI_LIBRARIES}
        ${Ice_LIBRARIES}
	${LIBS_DIR}/libJderobotInterfaces.so
	${LIBS_DIR}/libjderobotutil.so)
