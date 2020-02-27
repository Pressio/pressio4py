#!/bin/bash

envScript=$HOME/Desktop/work/ROM/env_scripts/setenv_serial_gcc920.sh
source ${envScript}

WORKPATH="$HOME/Desktop/buildTest"
PRESSIOPATH="${WORKPATH}/pressio/install/include"
EIGENPATH="${WORKPATH}/eigen/install/include/eigen3"
PYBIND11PATH="${WORKPATH}/pybind11/install"

SRCDIR=/Users/fnrizzi/Desktop/work/ROM/gitrepos/pressio4py
PFX=${PWD}/install
bdirname=build

rm -rf ${bdirname} && mkdir ${bdirname} && cd ${bdirname}
cmake -DCMAKE_VERBOSE_MAKEFILE:BOOL=TRUE \
      -DCMAKE_INSTALL_PREFIX=${PFX} \
      -DCMAKE_CXX_COMPILER=${CXX} \
      -DCMAKE_BUILD_TYPE=Debug \
      -DEIGEN_INCLUDE_DIR=${EIGENPATH} \
      -DPRESSIO_INCLUDE_DIR=${PRESSIOPATH} \
      -DPYBIND11_DIR=${PYBIND11PATH} \
      ${SRCDIR}
make
cd ..

#-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=${PFX} \