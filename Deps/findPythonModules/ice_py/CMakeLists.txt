set(module "Ice")
find_python_module(${module} REQUIRED)

string(TOUPPER ${module} module_upper)

if(PY_${module_upper})
	SET (ICE_PY_MODULES  PY_${module_upper})
	list(APPEND DEPS libbz2-dev libssl-dev)
	list(APPEND DEPS_libbz2-dev libssl-dev)
endif(PY_${module_upper})
