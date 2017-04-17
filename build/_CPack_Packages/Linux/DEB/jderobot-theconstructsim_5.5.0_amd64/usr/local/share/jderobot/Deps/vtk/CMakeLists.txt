find_package(VTK)

if (VTK_FOUND)
	message("***VTK FOUND ${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}")
else()
	message ("*** VTK NOT FOUND")
endIF()
