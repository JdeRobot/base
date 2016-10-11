FIND_PATH( NODEJS_DIR nodejs node
               PATHS /usr/local/bin /usr/bin
               ENV PATH)

SET(NODEJS_COMPILE TRUE)

IF(NOT NODEJS_DIR)
	MESSAGE("*** NodeJS not found")
	SET(NODEJS_COMPILE FALSE)
ELSE()
	MESSAGE( STATUS " NodeJS found" )
ENDIF()

