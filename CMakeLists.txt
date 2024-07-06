#
# MIT License
#
# Copyright (c) 2024 Stellacore Corporation
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#


cmake_minimum_required(VERSION 3.13)

# Get source code identifier (e.g. hash code from git repo)
include(./cmake/func_setReleaseTag.cmake)
setReleaseTag(aSourceIdentity)

project(
	OriNet
	VERSION 0.1.0
	DESCRIPTION "OriNet - Computations for rigid body orientation networks."
	HOMEPAGE_URL
		"https://github.com/Stellacore/OriNet"
	LANGUAGES CXX
	)

set(thisProjLib "${PROJECT_NAME}")

set(thePackageDescription
	[[
	The 'OriNet' package contains C++ code for manipulating rigid
	body orientation data such as needed for survey or photogrammetric
	network computations.
	Capabilities in this initial version focus on robust determination
	of orientation between two bodies when multiple orientation
	measurements are available and are assumed to be affected by
	outlier data.
	]]
	)
set(thePackageVendor "Stellacore Corporation")
set(thePackageContact ${PROJECT_HOMEPAGE_URL})

include(CTest)
enable_testing()

include(GNUInstallDirs) # create and set CMAKE_INSTALL_<dir> vars

set(CMAKE_CXX_STANDARD 20)  # 20 req'd for graaflib, else 17 is okay
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(cmake/BuildFlags.cmake)

find_package(Rigibra REQUIRED NO_MODULE)
message(Rigibra Found: ${Rigibra_FOUND})
message(Rigibra Version: ${Rigibra_VERSION})

find_package(Engabra REQUIRED NO_MODULE)
message(Engabra Found: ${Engabra_FOUND})
message(Engabra Version: ${Engabra_VERSION})

message("### CMAKE_MAJOR_VERSION: " ${CMAKE_MAJOR_VERSION})
message("### CMAKE_MINOR_VERSION: " ${CMAKE_MINOR_VERSION})
message("### CMAKE_PATCH_VERSION: " ${CMAKE_PATCH_VERSION})
message("### CMAKE_TWEAK_VERSION: " ${CMAKE_TWEAK_VERSION})
message("### PROJECT: " ${PROJECT})
message("### PROJECT_VERSION: " ${PROJECT_VERSION})

##
## Project structure
##

##
## -- project source tree description
##

set(EXPORT_ALIAS_NAME ${PROJECT_NAME}::${PROJECT_NAME})
set(EXPORT_TARGETS_NAME ${PROJECT_NAME}Targets)
set(EXPORT_FILENAME ${EXPORT_TARGETS_NAME}.cmake)
message("### EXPORT_ALIAS_NAME: " ${EXPORT_ALIAS_NAME})
message("### EXPORT_TARGETS_NAME: " ${EXPORT_TARGETS_NAME})
message("### EXPORT_FILENAME: " ${EXPORT_FILENAME})


add_subdirectory(cmake)
add_subdirectory(doc)
add_subdirectory(src) # project source code
add_subdirectory(test)
add_subdirectory(demo)

# ===
# === Packaging
# ===

set(CPACK_STRIP_FILES YES)

set(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PROJECT_DESCRIPTION}")
set(CPACK_PACKAGE_DESCRIPTION "${thePackageDescription}")
set(CPACK_PACKAGE_VENDOR "${thePackageVendor}")
set(CPACK_PACKAGE_CONTACT "${thePackageContact}")

set(CPACK_PACKAGE_INSTALL_DIRECTORY "${CPACK_PACKAGE_NAME}")
set(CPACK_VERBATIM_VARIABLES TRUE)

## NOTE: default CMAKE_INSTALL_PREFIX will be /usr/local on linux
set(CPACK_PACKAGING_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

	## Debian specific info
	## Useful info:
	##   https://decovar.dev/blog/2021/09/23/cmake-cpack-package-deb-apt
	## 
	set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS YES) # adds libc, libstdc dependents
	set(CPACK_DEBIAN_PACKAGE_MAINTAINER "${CPACK_PACKAGE_CONTACT}")
	set(APT_PACKAGE_DEPLIST
		#	"rigibra (>= ${Rigibra_VERSION})"
		#	"engabra (>= ${Engabra_VERSION})"
		)
	set(CPACK_DEBIAN_PACKAGE_DEPENDS ${APT_PACKAGE_DEPLIST})

set(CPACK_GENERATOR "DEB")

# Create cmake source code for building package
if(PROJECT_IS_TOP_LEVEL)
	# This command writes a CPackConfig.cmake file (at top of build tree)
	# using the previously defined packaging variable values.
	include(CPack)
endif(PROJECT_IS_TOP_LEVEL)

message("CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")

message("###")
message("### CMAKE_INSTALL_LIBDIR: " ${CMAKE_INSTALL_LIBDIR})
message("### CMAKE_INSTALL_BINDIR: " ${CMAKE_INSTALL_BINDIR})
message("### CMAKE_INSTALL_INCLUDEDIR: " ${CMAKE_INSTALL_INCLUDEDIR})
message("### CMAKE_INSTALL_DOCDIR: " ${CMAKE_INSTALL_DOCDIR})
message("###")
message("### CMAKE_INSTALL_FULL_LIBDIR: " ${CMAKE_INSTALL_FULL_LIBDIR})
message("### CMAKE_INSTALL_FULL_BINDIR: " ${CMAKE_INSTALL_FULL_BINDIR})
message("### CMAKE_INSTALL_FULL_INCLUDEDIR: " ${CMAKE_INSTALL_FULL_INCLUDEDIR})
message("### CMAKE_INSTALL_FULL_DOCDIR: " ${CMAKE_INSTALL_FULL_DOCDIR})
message("###")
message("### DEB_PACKAGE_ROOT: " ${DEB_PACKAGE_ROOT})
message("###")
