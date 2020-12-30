/*
//@HEADER
// ************************************************************************
//
// decoder.hpp
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

#ifndef PRESSIO4PY_PYBINDINGS_DECODER_HPP_
#define PRESSIO4PY_PYBINDINGS_DECODER_HPP_

namespace pressio4py{

// template <typename mytypes>
// pressio::mpl::enable_if_t< mytypes::fom_state_t::traits::rank == 1 >
// maybeBindApplyMappingForWrongLayout(pybind11::module & m)
// {
// }

// template <typename mytypes>
// pressio::mpl::enable_if_t< mytypes::fom_state_t::traits::rank >= 2 >
// maybeBindApplyMappingForWrongLayout(pybind11::module & m)
// {
//   using rom_native_state_t = typename mytypes::rom_native_state_t;
//   using fom_invalid_native_state_t = typename mytypes::fom_invalid_native_state_t;

//   /* Note that here we also bind the applyMapping for the unsupported fom_state
//      with wrong layout, such that if the user tries to do:
//      yFom = np.zeros(..., order='C')
//      decoder.applyMapping(yRom, yFom)
//      this will pick up the wrong code paths and throw an error.
//   */
//   decoderPy.def
//     ("applyMapping",
//      &decoder_t::template applyMapping
//      <rom_native_state_t, typename mytypes::fom_invalid_native_state_t>);
// }

template <typename mytypes>
void createDecoderBindings(pybind11::module & m, std::string className)
{
  using rom_native_state_t   = typename mytypes::rom_native_state_t;
  using fom_native_state_t   = typename mytypes::fom_native_state_t;
  using decoder_native_jac_t = typename mytypes::decoder_native_jac_t;
  using decoder_t	     = typename mytypes::decoder_t;
  using decoder_native_jac_wrong_layout_t = typename mytypes::decoder_native_jac_wrong_layout_t;

  pybind11::class_<decoder_t> decoderPy(m, className.c_str());

  /* for decoder we expose three constructors:
     1. for the linear case, we pass the native array with
	correct layout that matches the one inside the decoder class.
	This works fine and is supposed to be right.

     2. for the linear case, we pass the native array with
	the WRONG layout such that is called, this throws a runtime error on purpose.

     3. the custom case, where the user can pass a custom object.
   */

  // 1. pass the jacobian object, this implies a linear decoder
  decoderPy.def(pybind11::init<decoder_native_jac_t>());
  // 2. pass the jacobian with the wrong layout
  decoderPy.def(pybind11::init<decoder_native_jac_wrong_layout_t>());
  // 3.: user passes a python object, this yields an arbitrary mapper
  decoderPy.def(pybind11::init<pybind11::object, std::string>());

  // bind method to get address of the jacobian stored inside decoder
  decoderPy.def("jacobianAddress", &decoder_t::jacobianAddress);

  // method for applying the mapping
  decoderPy.def
    ("applyMapping",
     &decoder_t::template applyMapping<rom_native_state_t, fom_native_state_t>);

  // // when we have rank-3 states, we need to make sure the fom_native_state_t
  // // has the right layout such that it can be viewed from the c++ side and
  // // the operation is reflected on the numpy array in Python.
  // // For rank-1 states, it does not matter but for rank-3 it does matter
  // // since if fom_native_state_t has wrong layout, it will be changed on the
  // // c++ side but the origianl python object won't
  // maybeBindApplyMappingForWrongLayout<mytypes>(m);
}

}//end namespace pressio4py
#endif
