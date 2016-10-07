find_package(VTK REQUIRED)

if (VTK_FOUND)
	message("*** ${VTK_MAJOR_VERSION} ${VTK_MINOR_VERISON}")
else()
	message ("*** VTK NOT FOUND")
endIF()
