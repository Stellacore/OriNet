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


cmake_minimum_required(VERSION 3.25)

project(
	OriNetVnV
	VERSION 0.1.0
	DESCRIPTION "OriNet/VnV - OriNet Verification and Validation."
	HOMEPAGE_URL
		"https://github.com/Stellacore/OriNet/vnv"
	LANGUAGES CXX
	)

# For convenience, use build flags from OriNet, but these could be anything
include(../cmake/BuildFlags.cmake)

set(CMAKE_CXX_STANDARD 20)  # 20 needed for Graaf library
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# OriNet - main package
find_package(OriNet REQUIRED NO_MODULE)
message(OriNet Found: ${OriNet_FOUND})
message(OriNet Version: ${OriNet_VERSION})

# Rigibra - rigid body algebra package
find_package(Rigibra REQUIRED NO_MODULE)
message(Rigibra Found: ${Rigibra_FOUND})
message(Rigibra Version: ${Rigibra_VERSION})

# Engabra - engineering geometric algebra package
find_package(Engabra REQUIRED NO_MODULE)
message(Engabra Found: ${Engabra_FOUND})
message(Engabra Version: ${Engabra_VERSION})

# Graaf - graph data structure library
include(FetchContent)
FetchContent_Declare(
	graaflib
	GIT_REPOSITORY https://github.com/bobluppes/graaf.git
	GIT_TAG main
	)
# skip Graaf testing and benchmarks for speed fo build here
set(SKIP_TESTS ON)
set(SKIP_BENCHMARKS ON)
set(SKIP_EXAMPLES ON)
FetchContent_MakeAvailable(graaflib)


set(ProgNames

	SimNetwork

	)

foreach(ProgName ${ProgNames})

	add_executable(
		${ProgName}
		${ProgName}.cpp
		)

	target_compile_options(
		${ProgName}
		PRIVATE
			$<$<CXX_COMPILER_ID:Clang>:${BUILD_FLAGS_FOR_CXX_CLANG}>
			$<$<CXX_COMPILER_ID:GNU>:${BUILD_FLAGS_FOR_CXX_GCC}>
			$<$<CXX_COMPILER_ID:MSVC>:${BUILD_FLAGS_FOR_CXX_VISUAL}>
		)

	target_link_libraries(
		${ProgName}
		PRIVATE
			OriNet::OriNet
			Rigibra::Rigibra
			Engabra::Engabra
			Graaf::Graaf
		)

endforeach(ProgName ${ProgNames})

