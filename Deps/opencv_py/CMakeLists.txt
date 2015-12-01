set(module "cv2")
find_python_module(${module} REQUIRED)

string(TOUPPER ${module} module_upper)

if(PY_${module_upper})
	SET (OpenCV_PY_MODULES  PY_${module_upper})
	SET(DEPS "${DEPS} python-opencv,")
	SET(DEPS_DEV "${DEPS_DEV} python-opencv,")
endif(PY_${module_upper})
