find_package(VTK REQUIRED)

if (VTK_FOUND)
	message("***VTK FOUND ${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}")
	list(APPEND DEPS vtk7)
	list(APPEND DEPS_DEV vtk7)
else()
	message ("*** VTK NOT FOUND")
endIF()
