set(module "yaml")

usePython(2)
find_python_module(${module} REQUIRED)

string(TOUPPER ${module} module_upper)

if(PY_${module_upper})
	SET (yaml_PY_MODULES  PY_${module_upper})
	list(APPEND DEPS python-yaml)
	list(APPEND DEPS_DEV python-yaml)
endif(PY_${module_upper})