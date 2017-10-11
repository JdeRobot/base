set(module "cv2")

usePython(2)
find_python_module(${module} REQUIRED)

string(TOUPPER ${module} module_upper)

if(PY_${module_upper})
	SET (OpenCV_PY_MODULES  PY_${module_upper})
	#list(APPEND DEPS opencv-python)
	#list(APPEND DEPS_DEV opencv-python)
endif(PY_${module_upper})
