
message("")
message("${Magenta}----------------------------------${ColourReset}")
message("${Magenta} Processing regression tests ${ColourReset}")
message("${Magenta}----------------------------------${ColourReset}")

# glob over current dir and use relative to keep the dir name without full path
file(GLOB allDirs RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} REGEX "*steady*")

# loop over each test directory
foreach(d ${allDirs})
  message("configuring test : ${d}")

  set(testSrcDir      ${CMAKE_CURRENT_SOURCE_DIR}/${d})
  set(testDestDir     ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${d})
  # make a corresponding directory inside build
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
	set(f2 "test_${d}_${fname}")
	# copy to destination
	configure_file(${f} ${testDestDir}/${f2} COPYONLY)
      else()
	# if the file is NOT a mian but something else just copy
	configure_file(${f} ${testDestDir}/${fname} COPYONLY)
      endif()
    endif()
  endforeach(f ${thisTestFiles})

endforeach(d ${allDirs})
message("")
