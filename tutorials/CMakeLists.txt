
message("${Magenta}----------------------------------${ColourReset}")
message("${Magenta} Processing tutorials ${ColourReset}")
message("${Magenta}----------------------------------${ColourReset}")

# create the tutorials subdirectory
file(MAKE_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/tutorials")

# need to copy to destination all auxiliary python files in here
file(GLOB auxPyFiles RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} REGEX "*.py*")
foreach(f ${auxPyFiles})
  configure_file(${f} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/tutorials/ COPYONLY)
endforeach()

# glob over current subdirectories and use relative to keep the dir name without full path
# use *tut* regex to get all subdirs that contain the substring "tut"
file(GLOB allDirs RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} REGEX "*tut*")

# create a file inside the build directory to run all tutorials
# in this file we append the exact command to run all tutorials sequentially
set(runAllTutorialsFile ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/runalltutorials.sh)
file(TOUCH ${runAllTutorialsFile})

# loop over each test directory
set(counter 0)
foreach(d ${allDirs})
  message("configuring tutorial : ${d}")
  MATH(EXPR counter "${counter}+1")

  set(testSrcDir      ${CMAKE_CURRENT_SOURCE_DIR}/${d})
  set(testDestDir     ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/tutorials/${d})
  # make a corresponding directory inside build
  message(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
  file(MAKE_DIRECTORY ${testDestDir})

  # explore all files inside the testSrcDir we are processing
  file(GLOB thisTestFiles "${testSrcDir}/*")
  foreach(f ${thisTestFiles})
    # get just the filename
    get_filename_component(fname ${f} NAME)

    # don't deal with silly files
    if (NOT f MATCHES ".DS_Store")
      if(f MATCHES "main*")

	# if the file is a main file,
	# change the filename to make it a test recognized by pytest,
	# prepend the test_ and also the folder name to make it unique
	set(f2 "${d}_${fname}")
	# copy to destination
	configure_file(${f} ${testDestDir}/${f2} COPYONLY)

	# to populate the runalltutorials script, we need to check if
	# the loop counter==1, if so we create the file, if not we append
	if (${counter} EQUAL 1)
	  file(WRITE ${runAllTutorialsFile} "python ${testDestDir}/${f2}\n")
	else()
	  file(APPEND ${runAllTutorialsFile} "python ${testDestDir}/${f2}\n")
	endif()

      else()

	# if the file is NOT a main but something else just copy
	configure_file(${f} ${testDestDir}/${fname} COPYONLY)

      endif()
    endif()
  endforeach(f ${thisTestFiles})

endforeach(d ${allDirs})
message("")
