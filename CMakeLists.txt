
cmake_minimum_required(VERSION 3.11.0)
project(pressio4py CXX)

include_directories(${EIGEN_INCLUDE_DIR} ${PRESSIO_INCLUDE_DIR})

find_package(pybind11 REQUIRED PATHS ${PYBIND11_DIR}/share/cmake)

pybind11_add_module(pressio4pyGalerkin src/rom/galerkin.cc)
#pybind11_add_module(pressio4pyLspg src/rom/lspg.cc)
#pybind11_add_module(pressio4pyTesting src/rom/testing.cc)
