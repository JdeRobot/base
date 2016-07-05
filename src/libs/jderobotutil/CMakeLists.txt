include_directories(${SLICE_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories(${INTERFACES_CPP_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories(${LIBS_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++

add_library (jderobotutil STATIC jderobotutil.h observer.cpp observer.h paramdict.cpp paramdict.h time.cpp time.h uncopyable.h CameraTask.h CameraTask.cpp CameraHandler.cpp CameraHandler.h)

add_library (jderobotutilshare SHARED jderobotutil.h observer.cpp observer.h paramdict.cpp paramdict.h time.cpp time.h uncopyable.h CameraTask.h CameraTask.cpp CameraHandler.cpp CameraHandler.h)

set_target_properties(jderobotutilshare PROPERTIES OUTPUT_NAME jderobotutil)
