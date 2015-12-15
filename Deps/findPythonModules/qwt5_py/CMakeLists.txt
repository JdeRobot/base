
set(module "PyQt4.Qwt5")
find_python_module(${module} REQUIRED)

string(TOUPPER ${module} module_upper)

if(PY_${module_upper})
	SET (QWT5_PY_MODULES  PY_${module_upper})
	list(APPEND DEPS python-qwt5-qt4)
	list(APPEND DEPS_DEV python-qwt5-qt4)
endif(PY_${module_upper})
