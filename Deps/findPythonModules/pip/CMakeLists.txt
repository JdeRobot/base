set(module "pip")
find_python_module(${module} REQUIRED)

string(TOUPPER ${module} module_upper)

if(PY_${module_upper})
	SET (OpenCV_PY_MODULES  PY_${module_upper})
	list(APPEND DEPS "python3-pip")
	list(APPEND DEPS_DEV "python3-pip")
endif(PY_${module_upper})