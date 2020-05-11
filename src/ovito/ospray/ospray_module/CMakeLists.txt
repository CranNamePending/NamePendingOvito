## Copyright 2009-2019 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

# this subdirectory builds the actual ospray plugin into a library
# "libospray_module_<modulename>.so".
#
# This shared library can depend on libospray, but should _not_ assume
# that it is living in the same address space as the application: When
# running in MPI mode ospray will, upon encountering the
# ospLoadModule("<moduleName>"), actually load and use this library on
# every worker node (which will typically not run the application). As
# such, the library should _not_ "commuincate" with the application
# through global variables or globally defined functions, and instead
# communicate solely through the ospNewGeometry(...), ospSet...(),
# ospCommit(...) etc functionality provided by ospray.


# build the actual shared library that defines the geometry.
#
# Note the name is important: In order for ospray to properly find and
# initialize a module referenced by a call to
# "ospLoadModule(<moduleName>) this module _has_ to
#
# a) be called libospray_module_<modulename>.so, and
# b) contain a (extern C linkage) initialization routine named
#    OSPError ospray_module_init_<moduleName>(int16_t versionMajor,
#        int16_t versionMinor, int16_t versionPatch)
#
INCLUDE(${OSPRAY_USE_FILE})
#FIND_PACKAGE(ospray 2.1.0 REQUIRED)
#message("TITI : ${OSPRAY_USE_FILE} and ${ospray_FOUND} ${OSPRAY_INCLUDE_DIR}")
INCLUDE_DIRECTORIES("${EMBREE_INCLUDE_DIRS}")
FIND_PACKAGE(ospcommon REQUIRED)

add_library(ospray_module_ovito SHARED)
INCLUDE(${OSPRAY_SDK_USE_FILE})

ospcommon_configure_build_type()

ispc_target_add_sources(ospray_module_ovito
        # the cpp file that contains all the plugin code - parsing
        # parameters in ospCommit(), creating and registering the object,
        # building accel structures, etc
        geometry/Discs.cpp
        geometry/Cones.cpp
        geometry/Quadrics.cpp
        geometry/Cylinders.cpp

        # the ispc file that contains the plugins for all vector code - ie,
        # for ray-primitive intersection and 'postIntersect' (reporting info
        # on a previously computed ray-prim intersection)
        geometry/Discs.ispc
        geometry/Cones.ispc
        geometry/Quadrics.ispc
        geometry/Cylinders.ispc

        # and finally, the module init code (not doing much, but must be there)
        moduleInit.cpp
		)

message("output ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

GET_TARGET_PROPERTY(OSPRAY_COMMON_LIBRARY ospcommon::ospcommon LOCATION)
message("OSPCOMMON MODULE : ${OSPRAY_COMMON_LIBRARY}")

ispc_include_directories(${ISPC_INCLUDE_DIR})
ispc_include_directories(${OSPRAY_ISPC_MODULE_INCLUDE_DIRS})
ispc_include_directories(${EMBREE_INCLUDE_DIRS})

TARGET_LINK_LIBRARIES(ospray_module_ovito PRIVATE "${EMBREE_LIBRARY}")
TARGET_LINK_LIBRARIES(ospray_module_ovito  PUBLIC "${OSPRAY_COMMON_LIBRARY}")
TARGET_LINK_LIBRARIES(ospray_module_ovito  PUBLIC ospray::ospray ospray::ospray_module_ispc ospray::ospray_sdk)
#SET_TARGET_PROPERTIES(ospray_module_ovito PROPERTIES MACOSX_RPATH TRUE)
IF(WIN32)
	TARGET_COMPILE_DEFINITIONS(ospray_module_ovito PRIVATE "NOMINMAX=")
ENDIF()

IF(NOT APPLE)
	# Move the OSPRay shared libraries to the directory where the OSPRay extension module is being built.
	# This is needed, because at runtime OSPRay will search for the extension module in directory containing libospray itself.
	GET_TARGET_PROPERTY(OSPRAY_LIBRARY ospray::ospray LOCATION)
	GET_TARGET_PROPERTY(OSPRAY_COMMON_LIBRARY ospcommon::ospcommon LOCATION)
	GET_TARGET_PROPERTY(OSPRAY_MODULE_ISPC_LIBRARY ospray::ospray_module_ispc LOCATION)
	message("ospray : ${OSPRAY_MODULE_ISPC_LIBRARY}" )
	OVITO_INSTALL_SHARED_LIB("${OSPRAY_LIBRARY}" ".")
	OVITO_INSTALL_SHARED_LIB("${OSPRAY_COMMON_LIBRARY}" ".")
	OVITO_INSTALL_SHARED_LIB("${OSPRAY_MODULE_ISPC_LIBRARY}" ".")
ELSE()
	INSTALL(CODE "
		EXECUTE_PROCESS(COMMAND \"\${CMAKE_COMMAND}\" -E create_symlink \"libospray_module_ispc.0.dylib\" \"\${CMAKE_INSTALL_PREFIX}/${OVITO_RELATIVE_3RDPARTY_LIBRARY_DIRECTORY}/libospray_module_ispc.dylib\")
	")
ENDIF()

SET_TARGET_PROPERTIES(ospray_module_ovito PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${Ovito_BINARY_DIR}/${OVITO_RELATIVE_3RDPARTY_LIBRARY_DIRECTORY}")
SET_TARGET_PROPERTIES(ospray_module_ovito PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${Ovito_BINARY_DIR}/${OVITO_RELATIVE_3RDPARTY_LIBRARY_DIRECTORY}")

# Install this target.
INSTALL(TARGETS ospray_module_ovito EXPORT OVITO
	RUNTIME DESTINATION "${OVITO_RELATIVE_3RDPARTY_LIBRARY_DIRECTORY}"
	LIBRARY DESTINATION "${OVITO_RELATIVE_3RDPARTY_LIBRARY_DIRECTORY}"
	ARCHIVE DESTINATION "${OVITO_RELATIVE_3RDPARTY_LIBRARY_DIRECTORY}" COMPONENT "development")
message("module ovito install destination : ${OVITO_RELATIVE_3RDPARTY_LIBRARY_DIRECTORY}")
