
set(module "numpy")
find_python_module(${module} REQUIRED)

string(TOUPPER ${module} module_upper)

if(PY_${module_upper})
	SET (NUMPY_MODULES  PY_${module_upper})
endif(PY_${module_upper})
