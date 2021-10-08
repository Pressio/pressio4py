/*
//@HEADER
// ************************************************************************
//
// fom_steady_wrapper.hpp
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

#ifndef PRESSIO4PY_PYBINDINGS_ROM_FOM_STEADY_WRAPPER_HPP_
#define PRESSIO4PY_PYBINDINGS_ROM_FOM_STEADY_WRAPPER_HPP_

namespace pressio4py{ namespace rom{

template<
  typename scalar_t,
  typename state_t,
  typename residual_t,
  typename dense_matrix_t
  >
class FomWrapperSteadyState
{
  pybind11::object pyObj_;

public:
  using scalar_type       = scalar_t;
  using state_type	  = state_t;
  using residual_type	  = residual_t;

public:
  FomWrapperSteadyState() = delete;
  FomWrapperSteadyState(const FomWrapperSteadyState &) = default;
  FomWrapperSteadyState & operator=(const FomWrapperSteadyState &) = default;
  ~FomWrapperSteadyState() = default;

  explicit FomWrapperSteadyState(pybind11::object pyObj)
    : pyObj_(pyObj){}

public:
  residual_type createResidual() const{
    return pyObj_.attr("createResidual")();
  }

  dense_matrix_t createApplyJacobianResult(const dense_matrix_t & B) const{
    return pyObj_.attr("createApplyJacobianResult")(B);
  }

  void residual(const state_type & state,
		residual_type & r) const
  {
    pyObj_.attr("residual")(state, r);
  }

  void applyJacobian(const state_type & state,
		     const dense_matrix_t & operand,
		     dense_matrix_t & result) const
  {
    pyObj_.attr("applyJacobian")(state, operand, result);
  }
};

}}//namespace pressio4py::rom
#endif
