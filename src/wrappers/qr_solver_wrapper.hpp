/*
//@HEADER
// ************************************************************************
//
// qr_solver_wrapper.hpp
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

#ifndef PRESSIO4PY_PYBINDINGS_QR_SOLVER_WRAPPER_HPP_
#define PRESSIO4PY_PYBINDINGS_QR_SOLVER_WRAPPER_HPP_

namespace pressio4py{

template<typename matrix_t>
class QrSolverWrapper
{
  pybind11::object pyObj_;

public:
  using matrix_type = matrix_t;

public:
  QrSolverWrapper(pybind11::object pyObj)
    : pyObj_(pyObj){}

  QrSolverWrapper() = delete;
  QrSolverWrapper(const QrSolverWrapper &) = default;
  QrSolverWrapper & operator=(const QrSolverWrapper &) = default;
  ~QrSolverWrapper() = default;

public:
  void computeThin(const matrix_type & A)
  {
    pyObj_.attr("computeThin")(A);
  }

  template<typename r_type, typename state_type>
  void applyQTranspose(const r_type & operand, state_type & result) const
  {
    pyObj_.attr("applyQTranspose")(operand, result);
  }

  template<typename state_type>
  void applyRTranspose(const state_type & operand, state_type & result) const
  {
    pyObj_.attr("applyRTranspose")(operand, result);
  }

  template<typename state_type>
  void solve(const state_type & operand, state_type & result) const
  {
    pyObj_.attr("solveRxb")(operand, result);
  }
};

}//namespace pressio4py
#endif
