set(module "matplotlib")
find_python_module(${module} REQUIRED)

string(TOUPPER ${module} module_upper)

if(PY_${module_upper})
	SET (MATPLOTLIB_PY_MODULES  PY_${module_upper})
	SET(DEPS "${DEPS} python-matplotlib,")
	SET(DEPS_DEV "${DEPS_DEV} python-matplotlib,")
endif(PY_${module_upper})
