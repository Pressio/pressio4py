cmake_minimum_required(VERSION 3.18.0 FATAL_ERROR)
project(pressio4py LANGUAGES CXX)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(colors)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
  message(FATAL_ERROR "In-source builds are not allowed. Please use a separate build directory.")
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

set(PYBIND11_FINDPYTHON ON CACHE BOOL "" FORCE)
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/pressio/include
  ${CMAKE_CURRENT_SOURCE_DIR}/pressio-ops/include
  ${CMAKE_CURRENT_SOURCE_DIR}/pressio-log/include
)

set(PRESSIO4PY_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

if(NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
endif()

execute_process(
  COMMAND git rev-parse --abbrev-ref HEAD
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE PFPY_BRANCH
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET)

execute_process(
  COMMAND git rev-parse HEAD
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE PFPY_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET)

set(modulename _p4pyimpl)
pybind11_add_module(${modulename} ${PRESSIO4PY_SRC_DIR}/main_binder.cc)
target_compile_definitions(${modulename} PRIVATE MODNAME=${modulename} PRESSIO_ENABLE_LOGGING=1)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  target_compile_definitions(${modulename} PRIVATE PRESSIO_LOG_ACTIVE_MIN_LEVEL=2)
else()
  target_compile_definitions(${modulename} PRIVATE PRESSIO_LOG_ACTIVE_MIN_LEVEL=0)
endif()

install(
  TARGETS ${modulename}
  LIBRARY DESTINATION pressio4py
  RUNTIME DESTINATION pressio4py
  ARCHIVE DESTINATION pressio4py
)

message("${Yellow}----------------------------------${ColourReset}")
message("${Yellow} Pressio4Py config info: ${ColourReset}")
message("${Yellow}----------------------------------${ColourReset}")
message("${Yellow} - C++ standard   = ${CMAKE_CXX_STANDARD} ${ColourReset}")
message("${Yellow} - p4py branch    = ${PFPY_BRANCH} ${ColourReset}")
message("${Yellow} - p4py hash      = ${PFPY_HASH} ${ColourReset}")
message("${Yellow} - Build dir      = ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} ${ColourReset}")
message("${Yellow} - prefix         = ${CMAKE_INSTALL_PREFIX} ${ColourReset}")

add_subdirectory(tests_compiled)
