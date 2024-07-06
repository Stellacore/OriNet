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

# Generate API documentation with 'doxygen' tool
find_package(Doxygen)
if (DOXYGEN_FOUND)

	# Variables for expansion in Doxyfile template
	set(DoxyProjName "${PROJECT_NAME}")
	set(DoxyProjVersion "${PROJECT_VERSION}")
	set(DoxyProjTagline "${PROJECT_DESCRIPTION}")

	# Default enable doxygen build option
	option(BUILD_DOC "Build API documentation" ON)

	# Edit template Doxyfile to insert project specific info
	set(DoxTemplate ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
	set(DoxOutFile ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
	configure_file(${DoxTemplate} ${DoxOutFile} @ONLY)

	# command to invoke doxygen build
	add_custom_target(
		docs ALL
		COMMAND ${DOXYGEN_EXECUTABLE} ${DoxOutFile}
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		COMMENT "Generating documentation with Doxygen utility"
		VERBATIM
		)

message("#### CMAKE_CURRENT_BINARY_DIR/html : "
	${CMAKE_CURRENT_BINARY_DIR}/html)
message("#### CMAKE_INSTALL_DOCDIR/${local_LIBTARGET} : "
	${CMAKE_INSTALL_DOCDIR}/${local_LIBTARGET})
message("#### CMAKE_INSTALL_DOCDIR : "
	${CMAKE_INSTALL_DOCDIR})
message("#### CPACK_PACKAGING_INSTALL_PREFIX : "
	${CPACK_PACKAGING_INSTALL_PREFIX})
#[[
#]]

	# Copy entire document directory contents to install destination
	install(
		DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html
		DESTINATION
			${CMAKE_INSTALL_DOCDIR}/${local_LIBTARGET}
		)

else (DOXYGEN_FOUND)

	message("Doxygen tool required for building library API documentation")

endif (DOXYGEN_FOUND)

