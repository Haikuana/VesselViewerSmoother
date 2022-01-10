# ----------------------------------------------------------------------------
# Root CMake file for nanoflann
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.1)

# Extract library version into "NANOFLANN_VERSION"
# -----------------------------------------------------
# Look for: "#define NANOFLANN_VERSION 0xABC"
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/include/nanoflann.hpp" STR_HPP)
string(REGEX MATCHALL "NANOFLANN_VERSION.*0x[0-9,A-F]+" CMAKE_VERSION_LINE "${STR_HPP}")
string(REGEX MATCHALL "0x[0-9,A-F]+" NANOFLANN_VERSION_HEX "${CMAKE_VERSION_LINE}")

string(REGEX REPLACE "0x(.).*" "\\1" NANOFLANN_VERSION_MAJOR "${NANOFLANN_VERSION_HEX}" )
string(REGEX REPLACE "0x.(.).*" "\\1" NANOFLANN_VERSION_MINOR "${NANOFLANN_VERSION_HEX}" )
string(REGEX REPLACE "0x..(.).*" "\\1" NANOFLANN_VERSION_PATCH "${NANOFLANN_VERSION_HEX}" )
mark_as_advanced(STR_HPP CMAKE_VERSION_LINE NANOFLANN_VERSION_HEX NANOFLANN_VERSION_MAJOR NANOFLANN_VERSION_MINOR NANOFLANN_VERSION_PATCH)

#project(nanoflann VERSION "${NANOFLANN_VERSION_MAJOR}.${NANOFLANN_VERSION_MINOR}.${NANOFLANN_VERSION_PATCH}")

message(STATUS "nanoflann version: ${NANOFLANN_VERSION_MAJOR}.${NANOFLANN_VERSION_MINOR}.${NANOFLANN_VERSION_PATCH}")
file(WRITE "${nanoflann_BINARY_DIR}/version" "${NANOFLANN_VERSION_MAJOR}.${NANOFLANN_VERSION_MINOR}.${NANOFLANN_VERSION_PATCH}")

# Enable a high level of warnings.
if (CMAKE_COMPILER_IS_GNUCXX)
	# The -Wno-long-long is required in 64bit systems when including sytem headers.
	# The -Wno-variadic-macros was needed for Eigen3, StdVector.h
	add_compile_options(-Wall -Wno-long-long -Wno-variadic-macros -O2 -mtune=native)
	# Workaround: Eigen <3.4 produces *tons* of warnings in GCC >=6. See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1221
	if (NOT ${CMAKE_CXX_COMPILER_VERSION} LESS "6.0")
		add_compile_options(-Wno-ignored-attributes -Wno-int-in-bool-context)
	endif()
endif()

if(MSVC)
	 add_definitions( "/W3 /D_CRT_SECURE_NO_WARNINGS /nologo" )
endif()

# Solution Folder options:
if (${CMAKE_SOURCE_DIR} STREQUAL ${PROJECT_SOURCE_DIR})
	set_property(GLOBAL PROPERTY USE_FOLDERS ON)
	set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")
endif()

add_definitions ( -DNANOFLANN_PATH="${CMAKE_SOURCE_DIR}" )

include(GNUInstallDirs)
# Set relative install directories
set(INSTALL_INCLUDE_DIR "${CMAKE_INSTALL_INCLUDEDIR}")
set(INSTALL_PKGCONFIG_DIR "${CMAKE_INSTALL_DATADIR}/pkgconfig")
set(INSTALL_CMAKE_DIR "${CMAKE_INSTALL_DATADIR}/cmake/${PROJECT_NAME}")
set(INSTALL_COPYRIGHT_DIR "${CMAKE_INSTALL_DOCDIR}")


# Define nanoflann lib (header-only)
add_library(nanoflann INTERFACE)

# Tell CMake which C++ features we need
target_compile_features(nanoflann
	INTERFACE
		cxx_auto_type
		cxx_decltype
		cxx_deleted_functions
)
target_include_directories(nanoflann
	INTERFACE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR}>)

install(TARGETS nanoflann
        EXPORT nanoflannTargets)

add_library(nanoflann::nanoflann ALIAS nanoflann)

# Examples
option(NANOFLANN_BUILD_EXAMPLES "Build examples" ON)
if(NANOFLANN_BUILD_EXAMPLES)
	add_subdirectory(examples)	
endif()

# Tests
option(NANOFLANN_BUILD_TESTS "Build unit tests" OFF)
if(NANOFLANN_BUILD_TESTS)
	enable_testing()
	add_subdirectory(tests)
endif()

