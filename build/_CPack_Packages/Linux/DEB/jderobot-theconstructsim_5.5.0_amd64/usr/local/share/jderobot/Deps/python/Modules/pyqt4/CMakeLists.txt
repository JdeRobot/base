
set(module "PyQt4")

usePython(2)
find_python_module(${module} REQUIRED)

string(TOUPPER ${module} module_upper)

if(PY_${module_upper})
	SET (QT4_PY_MODULES  PY_${module_upper})
	
endif(PY_${module_upper})
list(APPEND DEPS  python-qt4 python-qwt5-qt4)
list(APPEND DEPS_DEV python-qt4 python-qwt5-qt4)
