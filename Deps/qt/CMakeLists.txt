cmake_minimum_required(VERSION 2.8.11)

# due to https://www.kdab.com/using-cmake-with-qt-5/
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Svg REQUIRED)
find_package(Qt5OpenGL REQUIRED)



include_directories(${Qt5Widgets_INCLUDE_DIRS}
        ${Qt5Core_INCLUDE_DIRS}
        ${Qt5Gui_INCLUDE_DIRS}
        ${Qt5Svg_INCLUDE_DIRS}
        ${Qt5OpenGL_INCLUDE_DIRS}
)
add_definitions(${Qt5Widgets_DEFINITIONS}
                ${Qt5Core_DEFINITIONS}
                ${Qt5Gui_DEFINITIONS}
                ${Qt5Svg_DEFINITIONS}
                ${Qt5OpenGL_DEFINITIONS}
)

SET(QT5_LIBRARIES_JDE ${Qt5Widgets_LIBRARIES}
                        ${Qt5Core_LIBRARIES}
                        ${Qt5Gui_LIBRARIES}
                        ${Qt5Svg_LIBRARIES}
                        ${Qt5OpenGL_LIBRARIES}
)

if (QT5_LIBRARIES_JDE)
    SET(QT5_COMPILE TRUE)
                MESSAGE("*** QT LIBRARIES Found ${QT5_LIBRARIES_JDE}")

        list(APPEND DEPS libqt5core5a libqt5opengl5 libqt5declarative5 libqt5gui5 libqt5svg5 libqt5widgets5)
        list(APPEND DEPS_DEV libqt5opengl5-dev libqt5svg5-dev qtbase5-dev)
else()
    SET(QT5_COMPILE FALSE)
		MESSAGE("*** QT LIBRARIES NOT FOUND")
endif()

