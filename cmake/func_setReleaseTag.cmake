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

