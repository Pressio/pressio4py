
cmake_minimum_required(VERSION 3.11.0)
project(pressio4py CXX)

# setting where other cmake commands are
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# include commands for coloring prints
include(colors)

#=====================================================================
# we need c++14
#=====================================================================
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#=====================================================================
### guard against in-source builds ###
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds are not allowed.
    Please make a new directory (e.g. build directory) and run CMake from there.")
endif()

### default to release if build type is empty ###
if (NOT CMAKE_BUILD_TYPE)
  message(STATUS "${CMAKE_BUILD_TYPE} is not specified, defaulting to Release.")
  set(CMAKE_BUILD_TYPE "Release")
endif()

### check if PRESSIO_INCLUDE_DIR is set ###
if (NOT PRESSIO_INCLUDE_DIR)
  message(FATAL_ERROR "${PRESSIO_INCLUDE_DIR} is not specified, must be set.")
endif()

#----------------------------------------------------------------
include_directories(${PRESSIO_INCLUDE_DIR})
#find_package(pybind11 REQUIRED PATHS ${PYBIND11_DIR}/share/cmake)
add_subdirectory(pybind11)

#----------------------------------------------------------------
# dir where all sources for pressio4py are
set(PRESSIO4PY_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
# dir where all python apps are
set(PRESSIO4PY_APPS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/pressio4pyApps)

#----------------------------------------------------------------
if(NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
endif()

#----------------------------------------------------------------
# get the working branch
execute_process(
  COMMAND git rev-parse --abbrev-ref HEAD
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE PFPY_BRANCH
  OUTPUT_STRIP_TRAILING_WHITESPACE)

# get the commit hash of the pressio4py being used
execute_process(
  COMMAND git rev-parse HEAD
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  RESULT_VARIABLE SHORT_HASH_RESULT
  OUTPUT_VARIABLE PFPY_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE)

#----------------------------------------------------------------
# command to create the pressio4py library
pybind11_add_module(pressio4py ${PRESSIO4PY_SRC_DIR}/pressio4py.cc)
# VERSION_INFO is defined by setup.py and passed into the C++ code
target_compile_definitions(pressio4py PRIVATE VERSION_IN=${VERSION_INFO})

#=====================================================================
### display some info
#=====================================================================
message("${Yellow}----------------------------------${ColourReset}")
message("${Yellow} Pressio4Py config info: ${ColourReset}")
message("${Yellow}----------------------------------${ColourReset}")
message("${Yellow} - C++ standard   = ${CMAKE_CXX_STANDARD} ${ColourReset}")
message("${Yellow} - p4py version   = ${VERSION_INFO} ${ColourReset}")
message("${Yellow} - p4py branch    = ${PFPY_BRANCH} ${ColourReset}")
message("${Yellow} - p4py hash      = ${PFPY_HASH} ${ColourReset}")
message("${Yellow} - Build dir      = ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} ${ColourReset}")
message("${Yellow} - prefix         = ${CMAKE_INSTALL_PREFIX} ${ColourReset}")
#message("${Yellow} - Pressio include = ${PRESSIO_INCLUDE_DIR} ${ColourReset}")

## need to process the apps directory ##
message("")
message("${Magenta}----------------------------------${ColourReset}")
message("${Magenta} Processing apps for tests/demos ${ColourReset}")
message("${Magenta}----------------------------------${ColourReset}")
set(appsDestDir ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/pressio4pyApps)
file(MAKE_DIRECTORY ${appsDestDir})
file(GLOB allApps "${PRESSIO4PY_APPS_DIR}/*.py")
foreach(f ${allApps})
  get_filename_component(fname ${f} NAME)
  message("Copying : ${fname} to build dir")
  configure_file(${f} ${appsDestDir} COPYONLY)
endforeach()

# add unit tests (we have to do this because unit tests
# contain c++ code that is compiled too)
add_subdirectory(unit_tests)

message(${CMAKE_INSTALL_PREFIX})
# install
install(TARGETS pressio4py DESTINATION ${CMAKE_INSTALL_PREFIX})
