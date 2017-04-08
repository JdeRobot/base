set(module "matplotlib")

usePython(2)
find_python_module(${module} REQUIRED)

string(TOUPPER ${module} module_upper)

if(PY_${module_upper})
	SET (MATPLOTLIB_PY_MODULES  PY_${module_upper})
	list(APPEND DEPS python-matplotlib)
	list(APPEND DEPS_DEV python-matplotlib python3-matplotlib)
endif(PY_${module_upper})
