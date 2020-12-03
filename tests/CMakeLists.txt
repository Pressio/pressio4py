
message("")
message("${Magenta}----------------------------------${ColourReset}")
message("${Magenta} Processing apps for tests ${ColourReset}")
message("${Magenta}----------------------------------${ColourReset}")

set(appsDestDir ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/apps)
file(MAKE_DIRECTORY ${appsDestDir})
file(GLOB allApps "${PRESSIO4PY_APPS_DIR}/*.py")
foreach(f ${allApps})
  get_filename_component(fname ${f} NAME)
  message("Copying : ${fname} to build dir")
  configure_file(${f} ${appsDestDir} COPYONLY)
endforeach()

add_subdirectory(unit)
add_subdirectory(regression)
