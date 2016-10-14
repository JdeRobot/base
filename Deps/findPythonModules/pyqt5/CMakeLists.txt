
set(module "PyQt5")
find_python_module(${module} REQUIRED)

string(TOUPPER ${module} module_upper)

if(PY_${module_upper})
	SET (QT5_PY_MODULES  PY_${module_upper})
	list(APPEND DEPS  python3-pyqt5 python3-pyqt5.qtsvg)
	list(APPEND DEPS_DEV python3-pyqt5 python3-pyqt5.qtsvg)
endif(PY_${module_upper})
