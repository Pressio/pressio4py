/*
//@HEADER
// ************************************************************************
//
// fomreconstructor.hpp
//                     		  Pressio
//                         Copyright 2019
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

#ifndef PRESSIO4PY_PYBINDINGS_FOM_RECONSTRUCTOR_HPP_
#define PRESSIO4PY_PYBINDINGS_FOM_RECONSTRUCTOR_HPP_

namespace pressio4py{

template <typename decoder_types>
void bindFomReconstructor(pybind11::module & m, std::string className)
{
  using state_types	   = typename decoder_types::states_t;
  using rom_state_t	   = typename state_types::rom_state_t;
  using fom_state_t	   = typename state_types::fom_state_t;
  using fom_native_state_t = typename state_types::fom_native_state_t;
  using decoder_t	   = typename decoder_types::decoder_t;

  // fom reconstructor type
  using fom_reconstructor_t =
    pressio::rom::FomStateReconstructor<::pressio4py::scalar_t, fom_state_t, decoder_t>;

  // create actual class
  pybind11::class_<fom_reconstructor_t> fomReconstructor(m, className.c_str());
  // constructor
  fomReconstructor.def(pybind11::init<const fom_state_t &, const decoder_t &>());
  // constructor
  fomReconstructor.def(pybind11::init<const fom_native_state_t &,const decoder_t &>());

  // bind the evaluate method.
  // Here we have to use the template rom_state_t because
  // it allows the templated evaluate method of the fom reconstructor class
  // to know the rank of the rom state since this
  // should work for rank1 and rank2 rom states.
  // NOTE, however, that calling "evaluate" from python requires
  // one to pass a NATIVE numpy array.
  fomReconstructor.def("evaluate",
		       &fom_reconstructor_t::template evaluate<rom_state_t>);
}

}//end namespace pressio4py
#endif
