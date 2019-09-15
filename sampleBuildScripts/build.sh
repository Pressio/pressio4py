#!/bin/bash

envScript=$HOME/setenv_ompi400_clang700.sh
source ${envScript}

WORKPATH="$HOME/Desktop"
TPLSPATH="${WORKPATH}/clang/clang700_ompi400_debug_shared"
EIGENPATH="${TPLSPATH}/eigen/install/include/eigen3"
PYBIND11PATH="${TPLSPATH}/pybind11/install"
PRESSIOPATH="${WORKPATH}/clang/clang700_ompi400_debug_shared/pressio/install/include"

SRCDIR=${WORKPATH}/sources/pressio4py
PFX=${PWD}/install
bdirname=build

rm -rf ${bdirname} && mkdir ${bdirname} && cd ${bdirname}
cmake -DCMAKE_CXX_COMPILER=${CC} \
      -DCMAKE_CXX_COMPILER=${CXX} \
      -DEIGEN_INCLUDE_DIR=${EIGENPATH} \
      -DPYBIND11_DIR=${PYBIND11PATH} \
      -DPRESSIO_INCLUDE_DIR=${PRESSIOPATH} \
      -DCMAKE_LIBRARY_OUTPUT_DIRECTORY=${PFX} \
      ${SRCDIR}
make
cd ..
