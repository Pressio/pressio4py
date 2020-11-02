/*
//@HEADER
// ************************************************************************
//
// lin_solver_wrapper.hpp
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

#ifndef PRESSIO4PY_PYBINDINGS_LINEAR_SOLVER_WRAPPER_HPP_
#define PRESSIO4PY_PYBINDINGS_LINEAR_SOLVER_WRAPPER_HPP_

namespace pressio4py{

template<typename matrix_t>
class LinSolverWrapper
{
  static_assert
  (pressio::containers::predicates::is_dense_matrix_wrapper_pybind<matrix_t>::value,
   "matrix_t for LinSolverWrapper must be a dense matrix wrapper pybind");

  pybind11::object pyObj_;

public:
  using matrix_type = matrix_t;

public:
  LinSolverWrapper(pybind11::object pyObj)
    : pyObj_(pyObj){}

  LinSolverWrapper() = delete;
  LinSolverWrapper(const LinSolverWrapper &) = default;
  LinSolverWrapper & operator=(const LinSolverWrapper &) = default;
  LinSolverWrapper(LinSolverWrapper &&) = default;
  LinSolverWrapper & operator=(LinSolverWrapper &&) = default;
  ~LinSolverWrapper() = default;

public:
  template<typename state_type>
  pressio::mpl::enable_if_t<
  pressio::containers::predicates::is_vector_wrapper_pybind<state_type>::value
  >
  solve(const matrix_type & A,
	const state_type & b,
	state_type & x)
  {
    pyObj_.attr("solve")(*A.data(), *b.data(), *x.data());
  }
};

}//namespace pressio4py
#endif