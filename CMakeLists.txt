cmake_minimum_required(VERSION 2.8.12)
project(cmake_example)

set(EIGEN_INCLUDE_DIR "/Users/fnrizzi/Desktop/work/ROM/clang/clang700_ompi400_dbg_shared/eigen/install/include/eigen3")
set(ROMPP_INCLUDE_DIR "/Users/fnrizzi/Desktop/work/ROM/clang/clang700_ompi400_dbg_shared_eigen/rompp/install/include")

include_directories(${EIGEN_INCLUDE_DIR} ${ROMPP_INCLUDE_DIR})
add_subdirectory(pybind11)

#set(CMAKE_CC_COMPILER "/Users/fnrizzi/tpl/openmpi/4.0.0/install_clang700/bin/mpicc")
set(CMAKE_CXX_COMPILER /Users/fnrizzi/tpl/openmpi/4.0.0/install_clang700/bin/mpic++)


pybind11_add_module(pyrom src/rom/lspg.cc)
