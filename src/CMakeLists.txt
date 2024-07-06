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

##
## -- CMake build system description
##

message("### thisProjLib: ${thisProjLib}")


# ===
# === Library Components
# ===

# set(thisProjLibTarget "${thisProjLib}Lib")

set(srcFiles

	OriNet.cpp

	network.cpp

	)

add_library(
	${thisProjLib}
	${srcFiles}
	)
add_library(
	${PROJECT_NAME}::${PROJECT_NAME}
	ALIAS
		${thisProjLib}
	)

target_sources(
	${thisProjLib}
	PUBLIC
		FILE_SET cppHeaders
			TYPE HEADERS
			BASE_DIRS
				../include/
			FILES
				../include/OriNet/align.hpp
				../include/OriNet/compare.hpp
				../include/OriNet/OriNet
				../include/OriNet/random.hpp
				../include/OriNet/robust.hpp
	)

target_compile_options(
	${thisProjLib}
	PRIVATE
		$<$<CXX_COMPILER_ID:Clang>:${BUILD_FLAGS_FOR_CXX_CLANG}>
		$<$<CXX_COMPILER_ID:GNU>:${BUILD_FLAGS_FOR_CXX_GCC}>
		$<$<CXX_COMPILER_ID:MSVC>:${BUILD_FLAGS_FOR_CXX_VISUAL}>
	)

target_include_directories(
	${thisProjLib}
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
		$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/ # local source headers
	)

target_link_libraries(
	${thisProjLib}
	PRIVATE
		Rigibra::Rigibra
		Engabra::Engabra
	)

##
## == Export CMake info for use of these targets by other CMake projects
##



## This is substituted into xConfig.cmake.in and used for install Exports
set(EXPORT_FILENAME ${PROJECT_NAME}Targets.cmake)

configure_package_config_file(
	${CMAKE_SOURCE_DIR}/cmake/xConfig.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
	INSTALL_DESTINATION
		${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
	)

write_basic_package_version_file(
	${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
	VERSION
		${PROJECT_VERSION}
	COMPATIBILITY
		SameMajorVersion
	)

install(FILES
		${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
		${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
	DESTINATION
		${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
	)


install(TARGETS
		${thisProjLib}
	EXPORT
		${PROJECT_NAME}Targets
	FILE_SET cppHeaders
	)

install(EXPORT
		${PROJECT_NAME}Targets
	FILE
		${EXPORT_FILENAME}  # ${PROJECT_NAME}Targets.cmake & xConfig.cmake.in
	NAMESPACE
		${PROJECT_NAME}::
	DESTINATION
		${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
	)

