
set(module "numpy")
find_python_module(${module} REQUIRED)

string(TOUPPER ${module} module_upper)

if(PY_${module_upper})
	SET (NUMPY_MODULES  PY_${module_upper})
	list(APPEND DEPS "python3-numpy")
	list(APPEND DEPS_DEV "python3-numpy")
endif(PY_${module_upper})
