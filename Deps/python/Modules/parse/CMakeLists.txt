
set(module "parse")

usePython(2)
find_python_module(${module} REQUIRED)

string(TOUPPER ${module} module_upper)

if(PY_${module_upper})
	SET (${module_upper}_MODULES  PY_${module_upper})
endif(PY_${module_upper})
