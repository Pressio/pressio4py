/*
//@HEADER
// ************************************************************************
//
// pressio4py.hpp
//                     		  Pressio
//                             Copyright 2019
//    National Technology & Engineering Solutions of Sandia, LLC (NTESS)
//
// Under the terms of Contract DE-NA0003525 with NTESS, the
// U.S. Government retains certain rights in this software.
//
// Pressio is licensed under BSD-3-Clause terms of use:
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived
// from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
// IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact Francesco Rizzi (fnrizzi@sandia.gov)
//
// ************************************************************************
//@HEADER
*/

#ifndef PRESSIO4PY_PYBINDINGS_PRESSIO_FOUR_PY_HPP_
#define PRESSIO4PY_PYBINDINGS_PRESSIO_FOUR_PY_HPP_

#include <pybind11/pybind11.h>
#include "types.hpp"
#include "./rom/decoder.hpp"
#include "./rom/fomreconstructor.hpp"
#include "./rom/galerkin.hpp"
#include "./rom/lspg.hpp"

PYBIND11_MODULE(pressio4py, mParent)
{

  pybind11::module m1 = mParent.def_submodule("rom");

  createDecoderBindings<pressio4py::ROMTypes>(m1);
  createFomReconstructorBindings<pressio4py::ROMTypes>(m1);

  pybind11::module m2 = m1.def_submodule("galerkin");
  createGalerkinBindings<pressio4py::ROMTypes>(m2);

  pybind11::module m3 = m1.def_submodule("lspg");
  pybind11::module m4 = m3.def_submodule("unsteady");
  createUnsteadyLSPGBindings<pressio4py::ROMTypes>(m4);
}

#endif
