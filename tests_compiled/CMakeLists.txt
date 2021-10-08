
message("")
message("${Magenta}----------------------------------${ColourReset}")
message("${Magenta} Processing unit tests ${ColourReset}")
message("${Magenta}----------------------------------${ColourReset}")

# for unit tests, I need to include the source dir
include_directories(${PRESSIO4PY_SRC_DIR})

# create the unit tests subdir
set(testdir ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/tests_compiled)
file(MAKE_DIRECTORY ${testdir})
message("TESTDIR ${testdir}")

#--------------------------------
# unit tests
#--------------------------------
# 1. glob over *.py
file(GLOB PYTEST_FILES "*.py")

# 1. copy all files.py to testdir
foreach(f ${PYTEST_FILES})
  get_filename_component(fname ${f} NAME_WLE)
  message("configuring unit test : ${fname}")
  configure_file(${f} ${testdir} COPYONLY)
endforeach()

# 2. glob over all .cpp files
file(GLOB CPPTEST_FILES "*.cc")

foreach(f ${CPPTEST_FILES})
  # f contains full file name with path, e.g. blabla/test_wrapper.cc
  # Extract filename without extension: test_wrapper
  get_filename_component(cppfile ${f} NAME_WLE)

  # add module for this test: need to add _module to
  # make sure the module has different name than test file
  # otherwise pytest and pybind11 complain
  set(modname ${cppfile}_module)

  pybind11_add_module(${modname} ${f})
  set_target_properties(${modname} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${testdir})
endforeach()
