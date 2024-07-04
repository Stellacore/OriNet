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

# ===
# === Compilation and linkage flags
# ===


##
## C Flags (used as basis of C++ flags below)
##

set(BUILD_FLAGS_FOR_C_CLANG
	"-DOriNet_Project_Version=\"${PROJECT_VERSION}\""
	"-DOriNet_Source_Identity=\"${aSourceIdentity}\""
	#
	-pthread
	-fomit-frame-pointer
	-fPIC  # Important for python callable libraries
	# warnings
	-pedantic-errors
	-Wall
	-Wextra
	-Wuninitialized
	-Wno-psabi
	    # disable ABI compatibility warnings (about std::pair changes in C++17)
		#- but OK if everything compiled with same compiler
	)

set(BUILD_FLAGS_FOR_C_GCC
	"-DOriNet_Project_Version=\"${PROJECT_VERSION}\""
	"-DOriNet_Source_Identity=\"${aSourceIdentity}\""
	#
	# build differences
	-pthread
	-fomit-frame-pointer
	-fmax-errors=1  # keep template error message generation under control
	-fPIC  # Important for python callable libraries
	# warnings
	-pedantic-errors
	-Wall
	-Wextra
	-Wuninitialized
	-Winit-self
	-Wno-psabi
	    # disable ABI compatibility warnings (about std::pair changes in C++17)
		#- but OK if everything compiled with same compiler
	)

set(BUILD_FLAGS_FOR_C_VISUAL
	""
	)

##
## C++ Flags (Add onto C-flags)
##

set(BUILD_FLAGS_FOR_CXX_CLANG
	${BUILD_FLAGS_FOR_C_CLANG}
	# flags
	-fstrict-enums
	# warnings
	-Wc++11-compat
	)

set(BUILD_FLAGS_FOR_CXX_GCC
	# C-lang flags
	${BUILD_FLAGS_FOR_C_GCC}
	# flags
	-fno-operator-names
	-fstrict-enums
	-fno-nonansi-builtins
	# warnings
	-Wc++11-compat
	)
set(BUILD_FLAGS_FOR_CXX_VISUAL
	${BUILD_FLAGS_FOR_C_VISUAL}
	""
	)

