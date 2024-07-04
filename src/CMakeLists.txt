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

