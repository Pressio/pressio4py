#!/bin/bash

#############
#TO BE FIXED
#############



# MPIPATH=/Users/fnrizzi/tpl/openmpi/4.0.0/install_clang700
# export CC=${MPIPATH}/bin/mpicc
# export CXX=${MPIPATH}/bin/mpic++

# export WORKPATH="/Users/fnrizzi/Desktop/work/ROM/"
# export TPLSPATH="${WORKPATH}/clang/clang700_ompi400_dbg_shared"
# export EIGENPATH="${TPLSPATH}/eigen/install/include/eigen3"
# export ROMPPPATH="${WORKPATH}/clang/clang700_ompi400_dbg_shared_eigen/rompp/install/include"

# rm -rf build
# mkdir build
# cd build
# cmake -DCMAKE_CXX_COMPILER=${CC} \
#       -DCMAKE_CXX_COMPILER=${CXX} \
#       -DEIGEN_INCLUDE_DIR=${EIGENPATH} \
#       -DROMPP_INCLUDE_DIR=${ROMPPPATH} \
#       ../test
# make
# cd ..

#ROMPPPATH=/Users/fnrizzi/Desktop/work/ROM/clang/clang700_ompi400_dbg_shared_eigen
#TPLSPATH=/Users/fnrizzi/Desktop/work/ROM/clang/clang700_ompi400_dbg_shared
#EIGENPATH=${TPLSPATH}/eigen/install/include/eigen3
#ROMPPPATH=${ROMPPPATH}/rompp/install/include
#PYBIND=/Users/fnrizzi/Desktop/pybind/pybind11/include
#PYTHONPATH=/opt/local/Library/Frameworks/Python.framework/Versions/3.4/include/python3.4m/
#${CXX} -O3 -Wall -shared -std=c++11 -I${EIGENPATH} -I${ROMPPPATH} -I${PYBIND} -I${PYTHONPATH}  -fPIC example_bind.cc -o example
