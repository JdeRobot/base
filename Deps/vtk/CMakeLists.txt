find_package(VTK REQUIRED)

if (VTK_FOUND)
	message("***VTK FOUND ${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}")
else()
	message ("*** VTK NOT FOUND")
endIF()
