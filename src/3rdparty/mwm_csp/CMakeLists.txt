#######################################################################################
#
#  Copyright 2019 Alexander Stukowski and Technische Universität Darmstadt
#
#  This file is part of OVITO (Open Visualization Tool).
#
#  OVITO is free software; you can redistribute it and/or modify it either under the
#  terms of the GNU General Public License version 3 as published by the Free Software
#  Foundation (the "GPL") or, at your option, under the terms of the MIT License.
#  If you do not alter this notice, a recipient may use your version of this
#  file under either the GPL or the MIT License.
#
#  You should have received a copy of the GPL along with this program in a
#  file LICENSE.GPL.txt.  You should have received a copy of the MIT License along
#  with this program in a file LICENSE.MIT.txt
#
#  This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY KIND,
#  either express or implied. See the GPL or the MIT License for the specific language
#  governing rights and limitations.
#
#######################################################################################

# Build library.
ADD_LIBRARY(MinimumWeightMatchingCentrosymmetry
	matching.cpp
	mwm_csp.cpp
)

# Give our library file a new name to not confuse it with any system versions of the library.
SET_TARGET_PROPERTIES(MinimumWeightMatchingCentrosymmetry PROPERTIES OUTPUT_NAME "ovito_mwm_csp")

# Make header files of this library available to dependent targets.
TARGET_INCLUDE_DIRECTORIES(MinimumWeightMatchingCentrosymmetry INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/..")

IF(NOT BUILD_SHARED_LIBS AND OVITO_BUILD_PYTHON_PACKAGE)
	# Since we will link this library into the dynamically loaded Python extension module, we need to use the fPIC flag.
	SET_PROPERTY(TARGET MinimumWeightMatchingCentrosymmetry PROPERTY POSITION_INDEPENDENT_CODE ON)
ENDIF()

# Export this target.
INSTALL(TARGETS MinimumWeightMatchingCentrosymmetry EXPORT OVITO
	RUNTIME DESTINATION "${OVITO_RELATIVE_LIBRARY_DIRECTORY}"
	LIBRARY DESTINATION "${OVITO_RELATIVE_LIBRARY_DIRECTORY}"
	ARCHIVE DESTINATION "${OVITO_RELATIVE_LIBRARY_DIRECTORY}" COMPONENT "development")
