cmake_minimum_required (VERSION 3.0)

set (CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type.")
set_property (CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Release" "Debug")
set (CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/solver" CACHE PATH "Install prefix.")
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/modules/")

# In some versions of libxml2 and libxml++ there is a bug causing
# that the document will not validate even if it is in accordance with the schema.
option (VALIDATE_AGAINST_SCHEMA "Strongly recommended to enable this option, it enables the validation of input files against the xml schema." ON)
if (VALIDATE_AGAINST_SCHEMA)
	set (VALIDATE_INPUT_FILE_AGAINST_SCHEMA 1)
endif ()


project (solver VERSION 1.0)

set (ILP_SOLVER "LPSolve" CACHE STRING "Integer Linear Programming solver.")
set_property (CACHE ILP_SOLVER PROPERTY STRINGS "LPSolve" "Gurobi" "Cplex")
set (LIBRARY_LINKAGE "SHARED" CACHE STRING "Library linkage.")
set_property (CACHE LIBRARY_LINKAGE PROPERTY STRINGS "SHARED" "STATIC")

if ("${ILP_SOLVER}" STREQUAL "Cplex")
	set (CPLEX_SOLVER 1)
elseif ("${ILP_SOLVER}" STREQUAL "Gurobi")
	set (GUROBI_SOLVER 1)
elseif ("${ILP_SOLVER}" STREQUAL "LPSolve")
	set (LPSOLVE_SOLVER 1)
else ()
	message (FATAL_ERROR "Unsupported ILP solver!")
endif ()


if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
	# It is necessary to enable C++11 support for GCC or Clang compilers.
	add_definitions ("-std=c++11 -Wall -pedantic")
	# We have added '-static-libstdc++' linking flag
	# to workaround the bug with broken ABI of libstdc++ gcc libraries
	# (gcc-4.7, clang-3.3 using the libraries of gcc-4.8).
	# It was resulting in invalid time measurement if the
	# chrono extension of C++11 library was used.
	set (CMAKE_EXE_LINKER_FLAGS "-static-libstdc++" CACHE STRING "" FORCE)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
	# Enable C++11 support for Intel compiler.
	if (WIN32)
		add_definitions ("/Qstd=c++11")
	endif()
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
	# Enable exception handling for Microsoft MSVC compiler.
	add_definitions ("/EHsc")
else ()
	message (FATAL_ERROR "Unsupported compiler '${CMAKE_CXX_COMPILER_ID}'!")
endif()

option (ENABLE_OPENMP "It enables OpenMP, needs a compiler support." ON)
if (ENABLE_OPENMP)
	find_package (OpenMP)
	if (OPENMP_FOUND)
		set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
		set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	endif ()
endif ()

set (FILE_MACRO "__FILE__")
set (LINE_MACRO "__LINE__")

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
	set (FUNC_MACRO "__PRETTY_FUNCTION__")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
	set (FUNC_MACRO "__FUNCSIG__")
else ()
	set (FUNC_MACRO "__func__")
endif()

configure_file (
	"conf/SolverConfig.h.in"
	"${PROJECT_BINARY_DIR}/SolverConfig.h"
	)


# Set the correct path to the required libraries of the executable.
SET (CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib64")

# so that we will find all the header files
include_directories ("inc" "${PROJECT_BINARY_DIR}")
add_subdirectory ("src")

# Generating documentation, inspired from 'https://majewsky.wordpress.com/2010/08/14/tip-of-the-day-cmake-and-doxygen/'.
find_package (Doxygen)
if (DOXYGEN_FOUND)
	configure_file ("conf/doxyfilehtml.in" "${PROJECT_BINARY_DIR}/doxyfilehtml" @ONLY)
	add_custom_target (doc
		${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/doxyfilehtml
		WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
		COMMENT "Generating API documentation with Doxygen" VERBATIM
	)
	file (MAKE_DIRECTORY "${PROJECT_BINARY_DIR}/doc")
endif ()

# Install xml schema for input files verification.
install_files ("/share/${PROJECT_NAME}-${PROJECT_VERSION}/" FILES "xmlschema/dataset.xsd")

if (IS_DIRECTORY "${PROJECT_BINARY_DIR}/doc/")
	install (DIRECTORY "${PROJECT_BINARY_DIR}/doc" DESTINATION "share")
endif ()

