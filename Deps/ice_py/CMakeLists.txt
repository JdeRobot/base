set(module "Ice")
find_python_module(${module} REQUIRED)

string(TOUPPER ${module} module_upper)

if(PY_${module_upper})
	SET (ICE_PY_MODULES  PY_${module_upper})
	SET(DEPS "${DEPS} python-zeroc-ice,")
endif(PY_${module_upper})
