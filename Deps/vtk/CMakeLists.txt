find_package(VTK QUIET)

if (VTK_FOUND)
	message(STATUS "VTK v.${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION} found")
else()
	message (STATUS"${BoldYellow}VTK NOT FOUND${ColourReset}")
endIF()
