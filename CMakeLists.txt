# ************************************************************************
#
#                     		  Pressio
#                             Copyright 2019
#    National Technology & Engineering Solutions of Sandia, LLC (NTESS)
#
# Under the terms of Contract DE-NA0003525 with NTESS, the
# U.S. Government retains certain rights in this software.
#
# Pressio is licensed under BSD-3-Clause terms of use:
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
# IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Questions? Contact Francesco Rizzi (fnrizzi@sandia.gov)
#
# ************************************************************************

cmake_minimum_required(VERSION 3.11.0)
project(pressio4py CXX)

# guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds are not allowed.
    Please make a new directory (e.g. build directory) and run CMake from there.")
endif()

# default to relase if build type is empty
if (NOT CMAKE_BUILD_TYPE)
  message(STATUS "${CMAKE_BUILD_TYPE} is not specified, defaulting to Release.")
  set(CMAKE_BUILD_TYPE "Release")
endif()

# check if EIGEN_INCLUDE_DIR is set
if (NOT EIGEN_INCLUDE_DIR)
  message(FATAL_ERROR "${EIGEN_INCLUDE_DIR} is not specified, must be set.")
endif()
# check if PRESSIO_INCLUDE_DIR is set
if (NOT PRESSIO_INCLUDE_DIR)
  message(FATAL_ERROR "${PRESSIO_INCLUDE_DIR} is not specified, must be set.")
endif()
#----------------------------------------------------------------

include_directories(${EIGEN_INCLUDE_DIR} ${PRESSIO_INCLUDE_DIR})
find_package(pybind11 REQUIRED PATHS ${PYBIND11_DIR}/share/cmake)
#----------------------------------------------------------------

# define the dir where all sources for pressio4py are
set(PRESSIO4PY_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
# set dir where all python apps are
set(PRESSIO4PY_APPS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/apps)

# #----------------------------------------------------------------
if(NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
endif()
message("CMAKE_LIBRARY_OUTPUT_DIRECTORY = ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
# #----------------------------------------------------------------

# create the library 
pybind11_add_module(pressio4py ${PRESSIO4PY_SRC_DIR}/pressio4py.cc)
#----------------------------------------------------------------

add_subdirectory(tests)

message("The prefix dir is: ${CMAKE_INSTALL_PREFIX}")
install(TARGETS pressio4py DESTINATION ${CMAKE_INSTALL_PREFIX})
