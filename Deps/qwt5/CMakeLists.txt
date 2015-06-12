FIND_PATH( Qwt5_INCLUDE_DIR qwt.h 
               PATHS ${QT_INCLUDE_DIR} /usr/local/qwt/include /usr/include/qwt
               PATH_SUFFIXES qwt qwt5 qwt-qt4 qwt5-qt4 qwt-qt3 qwt5-qt3 include qwt/include qwt5/include qwt-qt4/include qwt5-qt4/include qwt-qt3/include qwt5-qt3/include ENV PATH)

FIND_LIBRARY( Qwt5_LIBRARY NAMES qwt5-qt4 qwt-qt4 qwt5 qwt PATHS /usr/local/qwt/lib /usr/local/lib /usr/lib ${QT_LIBRARY_DIR})

SET(QWT_COMPILE TRUE)

IF(NOT Qwt5_INCLUDE_DIR)
	MESSAGE("*** QWT_INCLUDE_DIR not set")
	SET(QWT_COMPILE FALSE)
ELSE()
	SET(QWT_INCLUDE_DIR ${Qwt5_INCLUDE_DIR})
	MESSAGE(STATUS "*** QWT_INCLUDE_DIR found")
ENDIF()

IF(NOT Qwt5_LIBRARY)
	MESSAGE("*** QWT_LIBRARY not set")
	SET(QWT_COMPILE FALSE)
ELSE()
	SET(QWT_LIBRARY ${Qwt5_LIBRARY})
	MESSAGE(STATUS "*** QWT_LIBRARY found")
ENDIF()

SET(qwt_libraries ${QWT_LIBRARY})

