/*
//@HEADER
// ************************************************************************
//
// decoder.hpp
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

#ifndef PRESSIO4PY_PYBINDINGS_DECODER_HPP_
#define PRESSIO4PY_PYBINDINGS_DECODER_HPP_

namespace pressio4py{ namespace rom{

void bindDecoder(pybind11::module & m)
{
  const std::string className = "Decoder";

  // using rom_state_t   = typename types::rom_state_t;
  // using fom_state_t   = typename types::fom_state_t;
  // using decoder_t     = typename types::decoder_t;
  // using decoder_jac_t = typename types::decoder_jac_t;
  // using decoder_jac_wrong_layout_t = typename types::decoder_jac_wrong_layout_t;

  // create the class
  pybind11::class_<decoder_t> decoderPy(m, className.c_str());

  /* -------------------------
     bind constructors:
     -------------------------

     For decoder, we bind 3 constructors:

     1. linear case: constructor accepting the array with
	correct layout that matches the one inside the decoder class.
	This works fine and is supposed to be correct.

     2. linear case: constructor accepting the array with
	the WRONG layout such that is called,
	this throws a runtime error on purpose.

     3. custom case: the user passes a custom object.
  */

  // 1. pass the jacobian with correct layout
  decoderPy.def(pybind11::init<typename decoder_t::jacobian_type>());

  // 2. pass the jacobian with the wrong layout
  //    note that this constructor is only added so that if the user
  //    tries to call this, it gets a runtime error.
  decoderPy.def(pybind11::init<typename decoder_t::wrong_layout_jacobian_type>());

  // 3.: user passes a python object, this yields an arbitrary mapper
  decoderPy.def(pybind11::init<pybind11::object, std::string>());

  /* -------------------------
     bind methods:
     ------------------------- */
  // bind method to get address of the jacobian stored inside decoder
  decoderPy.def("jacobianAddress", &decoder_t::jacobianAddress);

  // method for applying the mapping
  decoderPy.def("applyMapping",
		&decoder_t::template applyMapping<py_f_arr>);

  // // when we have rank-3 states, we need to make sure the fom_native_state_t
  // // has the right layout such that it can be viewed from the c++ side and
  // // the operation is reflected on the numpy array in Python.
  // // For rank-1 states, it does not matter but for rank-3 it does matter
  // // since if fom_native_state_t has wrong layout, it will be changed on the
  // // c++ side but the origianl python object won't
  // maybeBindApplyMappingForWrongLayout<types>(m);
}

}}//end namespace pressio4py
#endif
