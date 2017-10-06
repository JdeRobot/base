
set(module "imutils")

usePython(2)
find_python_module(${module})

string(TOUPPER ${module} module_upper)

if(PY_${module_upper})
	SET (IMUTILS_MODULES  PY_${module_upper})
endif(PY_${module_upper})
