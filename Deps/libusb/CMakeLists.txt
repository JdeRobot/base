pkg_check_modules(LIBUSB QUIET libusb-1.0)

if (LIBUSB_INCLUDE_DIRS)
	MESSAGE("-- libusb found")
	list(APPEND DEPS libusb-1.0-0)
	list(APPEND DEPS_DEV libusb-1.0-0-dev)
else()
	MESSAGE("-- ${BoldYellow}libusb not found${ColourReset}")
endif()


