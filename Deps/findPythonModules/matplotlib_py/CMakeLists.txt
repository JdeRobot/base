set(module "matplotlib")
find_python_module(${module} REQUIRED)

string(TOUPPER ${module} module_upper)

if(PY_${module_upper})
	SET (MATPLOTLIB_PY_MODULES  PY_${module_upper})
	list(APPEND DEPS python-matplotlib)
	list(APPEND DEPS_DEV python-matplotlib)
endif(PY_${module_upper})
