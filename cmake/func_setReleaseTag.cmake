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

#
# Set first argument to string suitable for use in project release branding
#

function(setReleaseTag ReleaseTagVar)

	# return value
	set (ResultString, "FooFooFoo")

	# message("func: ReleaseTagVar = ${ReleaseTagVar}")
	# message("func: \$\{${ReleaseTagVar}\} = '${${ReleaseTagVar}}'")


	if(${ReleaseTagVar})

		# If already set, then use the explicit version passed in
		message("### ReleaseTagVar: Using provided: '${ReleaseTagVar}'")
		#		set(ResultString ${ReleaseTagVar})

	else(${ReleaseTagVar})

		message("### ReleaseTagVar: Decoding Git Repository Description")

		# -- use timestamp
		# string (TIMESTAMP ResultString UTC)

		# -- use git describe info
		if(UNIX)
			set(GIT_CMD "git")
		endif(UNIX)
		if(WIN32)
			set(GIT_CMD "CMD /c git")
		endif(WIN32)

		# run git command to extract tag info
		execute_process(
			COMMAND ${GIT_CMD} describe
			OUTPUT_VARIABLE ResultString
			ERROR_VARIABLE ErrorString
			WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
			OUTPUT_STRIP_TRAILING_WHITESPACE
			)

		if("${ResultString}" STREQUAL "")
			message("###")
			message("###")
			message("### Can't get 'git describe' result")
			message("### ErrorString: ${ErrorString}")
			message("###")
			message("###")
			set(ResultString "NoSourceCodeIdentity_CantRunGitDescribe_!!")
		else()
			message("### ReleaseTagVar: ResultString ${ResultString}")
		endif()

	endif(${ReleaseTagVar})


	set (${ReleaseTagVar} ${ResultString} PARENT_SCOPE)

endfunction(setReleaseTag)

