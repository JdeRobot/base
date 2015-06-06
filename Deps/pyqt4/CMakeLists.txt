
set(module "PyQt4")
find_python_module(${module} REQUIRED)

string(TOUPPER ${module} module_upper)

if(PY_${module_upper})
	SET (QT4_PY_MODULES  PY_${module_upper})
	SET(DEPS "${DEPS}  python-qt4,")
endif(PY_${module_upper})
