FIND_PATH( QFI_INCLUDE_DIR qfi_ADI.h qfi_ALT.h qfi_HSI.h qfi_NAV.h qfi_PFD.h qfi_SI.h qfi_TC.h qfi_VSI.h 
               PATHS ${QT_INCLUDE_DIR} /usr/local/qfi/include /usr/include
               include ENV PATH)
FIND_LIBRARY( QFI_LIBRARY NAMES qfi PATHS /usr/local/lib /usr/lib)

SET(QFI_COMPILE TRUE)

IF(NOT QFI_INCLUDE_DIR)
	MESSAGE(STATUS "${BoldYellow}QFI includes not found${ColourReset}")
	SET(QFI_COMPILE FALSE)
ELSE()
	MESSAGE( STATUS "QFI includes found" )
ENDIF()

IF(NOT QFI_LIBRARY)
	MESSAGE(STATUS "${BoldYellow}QFI library not found${ColourReset}")
	SET(QFI_COMPILE FALSE)
ELSE()
	SET(QFI_LIBRARY ${QFI_LIBRARY})
	MESSAGE(STATUS "QFI library found")
	list(APPEND DEPS qfi)
	list(APPEND DEPS_DEV qfi)
ENDIF()

SET(QFI_libraries ${QFI_LIBRARY})


