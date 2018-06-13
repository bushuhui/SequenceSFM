
IF(WIN32)

	FIND_PATH( SUITESPARSE_PATH suitesparse/colamd.h
		PATHS
		$ENV{SUITESPARSE_PATH_HOME}
		C:/suitesparse/
	)
	
	if( SUITESPARSE_PATH )
	SET ( SUITESPARSE_FOUND 1 )
		SET( SUITESPARSE_INCLUDES "${SUITESPARSE_PATH}/include")
		
		MESSAGE( STATUS "Looking for SuiteSparse - found")
		MESSAGE( STATUS "SuiteSparse path: ${SUITESPARSE_PATH}" )
    endif( SUITESPARSE_PATH )
	
ELSE(WIN32) # Linux

	FIND_PATH( SUITESPARSE_PATH include/suitesparse/colamd.h
	    # system placed in /usr/local/include
        /usr/local
	    # system placed in /usr/include
        /usr
	)
	
	if( SUITESPARSE_PATH )
    	SET ( SUITESPARSE_FOUND 1 )
		SET( SUITESPARSE_INCLUDES "${SUITESPARSE_PATH}/include/suitesparse")
		
		MESSAGE( STATUS "Looking for SuiteSparse - found")
		MESSAGE( STATUS "SuiteSparse path: ${SUITESPARSE_INCLUDES}" )		
    endif( SUITESPARSE_PATH )
    

    set(SUITESPARSE_MODULES2FIND colamd)

	foreach (SUITESPARSE_MODULE_NAME ${SUITESPARSE_MODULES2FIND})
		FIND_LIBRARY(${SUITESPARSE_MODULE_NAME}_LIBRARIES NAMES ${SUITESPARSE_MODULE_NAME}
                        PATHS
                        /usr/lib/x86_64-linux-gnu
                        /usr/lib
                        /usr/local/lib
                        /opt/local/lib
			)
        
		if(${SUITESPARSE_MODULE_NAME}_LIBRARIES)
			set(${SUITESPARSE_MODULE_NAME}_INCLUDES ${SUITESPARSE_INCLUDES})
			set(${SUITESPARSE_MODULE_NAME}_FOUND 1)
			list(APPEND SUITESPARSE_LIBRARIES ${${SUITESPARSE_MODULE_NAME}_LIBRARIES})
		else(${SUITESPARSE_MODULE_NAME}_LIBRARIES)
			message("Can't found module " ${SUITESPARSE_MODULE_NAME})
		endif(${SUITESPARSE_MODULE_NAME}_LIBRARIES)
	endforeach()
ENDIF(WIN32)

IF(SUITESPARSE_FOUND)
    set(SuiteSparse_INCLUDES ${SUITESPARSE_INCLUDES} ${SUITESPARSE_PATH}/include)
    set(SuiteSparse_LIBRARIES ${SUITESPARSE_LIBRARIES})
ENDIF(SUITESPARSE_FOUND)
