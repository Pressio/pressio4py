
cmake_minimum_required(VERSION 2.8.12)
project(pressio4py)

include_directories(${EIGEN_INCLUDE_DIR} ${PRESSIO_INCLUDE_DIR})
find_package(pybind11 REQUIRED PATHS ${PYBIND11_DIR}/share/cmake)

pybind11_add_module(pressio4py src/bindings/rom/lspg.cc)

#configure_file(src/apps/burgers.py ${CMAKE_CURRENT_BINARY_DIR}/burgers.py COPYONLY)
#configure_file(src/ops/pressio4pyOps.py ${CMAKE_CURRENT_BINARY_DIR}/pressio4pyOps.py COPYONLY)
#configure_file(tests/lspg_burgers1d/test_burg_lspg.py ${CMAKE_CURRENT_BINARY_DIR}/test_burg_lspg.py COPYONLY)
#configure_file(tests/lspg_burgers1d/basis.txt ${CMAKE_CURRENT_BINARY_DIR}/basis.txt COPYONLY)
