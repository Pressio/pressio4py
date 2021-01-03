/*
//@HEADER
// ************************************************************************
//
// types.hpp
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

#ifndef PRESSIO4PY_PYBINDINGS_TYPES_HPP_
#define PRESSIO4PY_PYBINDINGS_TYPES_HPP_

// pressio include
#include "pressio_rom.hpp"

namespace pressio4py{

using scalar_t	= double;
using py_c_arr	= pybind11::array_t<scalar_t, pybind11::array::c_style>;
using py_f_arr	= pybind11::array_t<scalar_t, pybind11::array::f_style>;

template<int rank, typename = void>
struct StateTypes;

template<>
struct StateTypes<1>
{
  using rom_native_state_t	   = py_f_arr;
  using rom_state_t		   = pressio::containers::Tensor<1, rom_native_state_t>;
  using fom_native_state_t	   = py_f_arr;
  using fom_state_t		   = pressio::containers::Tensor<1, fom_native_state_t>;
};

template<int rank>
struct StateTypes<rank, pressio::mpl::enable_if_t<rank >= 2>>
{
  using rom_native_state_t	   = py_f_arr;
  using rom_state_t		   = pressio::containers::Tensor<rank, rom_native_state_t>;
  using fom_native_state_t	   = py_f_arr;
  using fom_invalid_native_state_t = py_c_arr;
  using fom_state_t		   = pressio::containers::Tensor<rank, fom_native_state_t>;
};
}//end namespace pressio4py

#include "./rom/wrappers/py_decoder.hpp"

namespace pressio4py{
template<int state_rank, int decoder_j_rank>
struct DecoderTypes : StateTypes<state_rank>
{
  static_assert
  (decoder_j_rank >= 2, "The rank of the decoder's jacobian should always be >= 2");

  using states_t = StateTypes<state_rank>;
  using typename states_t::fom_state_t;

  using decoder_native_jac_wrong_layout_t = py_c_arr;
  using decoder_native_jac_t = py_f_arr;
  using decoder_jac_t	     = pressio::containers::Tensor<decoder_j_rank, decoder_native_jac_t>;
  using decoder_t	     = pressio4py::PyDecoder<scalar_t, decoder_jac_t, fom_state_t>;
};

template<int state_rank, int decoder_j_rank>
struct GalerkinTypes : DecoderTypes<state_rank, decoder_j_rank>
{
  using typename DecoderTypes<state_rank, decoder_j_rank>::decoder_jac_t;
  using projector_t = pressio::rom::galerkin::impl::ArbitraryProjector<decoder_jac_t, void>;
};

namespace impl{
template<int state_rank=1, int decoder_j_rank=2>
struct LspgTypes : DecoderTypes<state_rank, decoder_j_rank>
{
  using typename DecoderTypes<state_rank, decoder_j_rank>::decoder_jac_t;
  using typename DecoderTypes<state_rank, decoder_j_rank>::decoder_native_jac_t;
  using lsq_hessian_t = pressio::containers::Tensor<2, decoder_native_jac_t>;
};
}

using LspgTypes = impl::LspgTypes<1,2>;

}//end namespace pressio4py
#endif
