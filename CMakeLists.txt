#--------------------------------------------------------------
# List all executables that do have no other dependencies
SET(FEA_DEMOS

)

SET(FEA_DEMOS_MKL
)

# List all executables that use Irrlicht
SET(FEA_DEMOS_IRR
    test_platform
)

# List all executables that use both Irrlicht and Matlab
SET(FEA_DEMOS_IRR_MATLAB

)

# List all executables that use both Irrlicht and MKL
SET(FEA_DEMOS_IRR_MKL

)

# List all executables that use Irrlicht, Matlab, and MKL
SET(FEA_DEMOS_IRR_MATLAB_MKL
)

INCLUDE_DIRECTORIES(CMAKE_CURRENT_SOURCE_DIR)

MESSAGE(STATUS "Demo programs for Platform module...")

#--------------------------------------------------------------
# Add executables for demos that have no other dependencies

FOREACH(PROGRAM ${FEA_DEMOS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")
    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    TARGET_LINK_LIBRARIES(${PROGRAM}
        ChronoEngine
        ChronoEngine_fea)
    ADD_DEPENDENCIES(${PROGRAM}
        ChronoEngine
        ChronoEngine_fea)
    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
ENDFOREACH()

#--------------------------------------------------------------
# Add executables for demos that depend only on MKL

FOREACH(PROGRAM ${FEA_DEMOS_MKL})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")
    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE} ${CH_MKL_LINK_FLAGS}")
    TARGET_LINK_LIBRARIES(${PROGRAM}
        ChronoEngine
		ChronoEngine_mkl
        ChronoEngine_fea)
	TARGET_INCLUDE_DIRECTORIES(${PROGRAM} PUBLIC
		${CH_MKL_INCLUDES}
	)
    ADD_DEPENDENCIES(${PROGRAM}
        ChronoEngine
		ChronoEngine_mkl
        ChronoEngine_fea)
    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
ENDFOREACH()

#--------------------------------------------------------------
# For demo executables that use Irrlicht
# (and possibly Matlab and/or MKL)

IF(ENABLE_MODULE_IRRLICHT)

    INCLUDE_DIRECTORIES( ${CH_IRRLICHTINC} )

    FOREACH(PROGRAM ${FEA_DEMOS_IRR})
        MESSAGE(STATUS "...add ${PROGRAM}")

        SET(SOURCE_FILES
          test_platform.cpp
          PlatformModel.cpp
          defineParameters.cpp
          MooringLine.cpp
          Buoyancy.cpp
        )

        ADD_EXECUTABLE(${PROGRAM}  ${SOURCE_FILES})

        SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")
        SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
            FOLDER demos
            COMPILE_FLAGS "-g ${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
            LINK_FLAGS "${CH_LINKERFLAG_EXE}")
        TARGET_LINK_LIBRARIES(${PROGRAM}
            ${CH_IRRLICHTLIB}
            ChronoEngine
            ChronoEngine_fea
            ChronoEngine_irrlicht)
        ADD_DEPENDENCIES(${PROGRAM}
            ChronoEngine ChronoEngine_fea ChronoEngine_irrlicht)
        INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})

    ENDFOREACH()

    IF(ENABLE_MODULE_MATLAB)
        INCLUDE_DIRECTORIES( ${CH_MATLABINC} )

        FOREACH(PROGRAM ${FEA_DEMOS_IRR_MATLAB})
            MESSAGE(STATUS "...add ${PROGRAM}")

            ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
            SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")
            SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
                FOLDER demos
                COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
                LINK_FLAGS "${CH_LINKERFLAG_EXE}")
            TARGET_LINK_LIBRARIES(${PROGRAM}
                ${CH_IRRLICHTLIB}
                ${CH_MATLABLIB}
                ChronoEngine
                ChronoEngine_fea
                ChronoEngine_irrlicht
                ChronoEngine_matlab)
            ADD_DEPENDENCIES(${PROGRAM}
                ChronoEngine
                ChronoEngine_fea
                ChronoEngine_irrlicht
                ChronoEngine_matlab)
            INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
        ENDFOREACH()
    ENDIF()

    IF(ENABLE_MODULE_MKL)
        INCLUDE_DIRECTORIES(${CH_MKL_INCLUDES})
        INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})

        FOREACH(PROGRAM ${FEA_DEMOS_IRR_MKL})
            MESSAGE(STATUS "...add ${PROGRAM}")

            ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
            SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")
            SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
                FOLDER demos
                COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_MKL_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
                LINK_FLAGS "${CH_LINKERFLAG_EXE} ${CH_MKL_LINK_FLAGS}")
            TARGET_LINK_LIBRARIES(${PROGRAM}
                ${CH_IRRLICHTLIB}
                ChronoEngine
                ChronoEngine_fea
                ChronoEngine_mkl
                ChronoEngine_irrlicht)
            ADD_DEPENDENCIES(${PROGRAM}
                ChronoEngine
                ChronoEngine_fea
                ChronoEngine_mkl
                ChronoEngine_irrlicht)
            INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
        ENDFOREACH()
    ENDIF()

    IF(ENABLE_MODULE_MATLAB AND ENABLE_MODULE_MKL)
        INCLUDE_DIRECTORIES( ${CH_MATLABINC} )

        INCLUDE_DIRECTORIES(${CH_MKLINC})
        INCLUDE_DIRECTORIES(${CH_EIGEN_PATH})
        INCLUDE_DIRECTORIES(${ChronoEngine_UNIT_MKL_INCLUDE})
        INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})

        FOREACH(PROGRAM ${FEA_DEMOS_IRR_MATLAB_MKL})
            MESSAGE(STATUS "...add ${PROGRAM}")

            ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
            SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")
            SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
                FOLDER demos
                COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_MKL_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
                LINK_FLAGS "${CH_LINKERFLAG_EXE} ${CH_MKL_LINK_FLAGS}")
            TARGET_LINK_LIBRARIES(${PROGRAM}
                ${CH_IRRLICHTLIB}
                ${CH_MATLABLIB}
                ChronoEngine
                ChronoEngine_fea
                ChronoEngine_mkl
                ChronoEngine_irrlicht
                ChronoEngine_matlab)
            ADD_DEPENDENCIES(${PROGRAM}
                ChronoEngine
                ChronoEngine_fea
                ChronoEngine_mkl
                ChronoEngine_irrlicht
                ChronoEngine_matlab)
            INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
        ENDFOREACH()
    ENDIF()

ENDIF()
