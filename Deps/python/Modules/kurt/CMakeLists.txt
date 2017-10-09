
set(module "kurt")

usePython(2)
find_python_module(${module})

string(TOUPPER ${module} module_upper)

if(PY_${module_upper})
	SET (KURT_MODULES  PY_${module_upper})
	list(APPEND DEPS kurt-jderobot)
	list(APPEND DEPS_DEV kurt-jderobot)
endif(PY_${module_upper})
