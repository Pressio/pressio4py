/*
//@HEADER
// ************************************************************************
//
// galerkin_solve_problem_api_bind.hpp
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

#ifndef PRESSIO4PY_PYBINDINGS_GALERKIN_SOLVE_PROBLEM_API_BIND_HPP_
#define PRESSIO4PY_PYBINDINGS_GALERKIN_SOLVE_PROBLEM_API_BIND_HPP_

namespace pressio4py
{

/*
  EXPLICIT
  metafunctions to facilitate binding api for the combinatorial case
  of multiple problems
*/
template<typename...>
struct bindGalerkinExplicitProbs;

template<class problem>
struct bindGalerkinExplicitProbs<std::tuple<problem>>
{
  template<class rom_state_t, class scalar_t, class collector_t>
  static void bind(pybind11::module & m)
  {
    m.def("advanceNSteps",
	  &pressio::rom::galerkin::solveNSteps<
	  problem, rom_state_t, scalar_t, collector_t>);
  }

  template<class rom_state_t, class scalar_t>
  static void bind(pybind11::module & m)
  {
    m.def("advanceNSteps",
	  &pressio::rom::galerkin::solveNSteps<
	  problem, rom_state_t, scalar_t>);
  }
};

template<class p1, class ... ptails>
struct bindGalerkinExplicitProbs<std::tuple<p1, ptails...>>
{
  template<class rom_state_t, class ...args>
  static void bind(pybind11::module & m)
  {
    bindGalerkinExplicitProbs
      <std::tuple<p1>>::template bind<rom_state_t, args...>(m);

    bindGalerkinExplicitProbs
      <std::tuple<ptails...>>::template bind<rom_state_t, args...>(m);
  }
};

//--------------------------------------------------------------------
/*
  IMPLICIT
  metafunctions to facilitate binding api for the combinatorial case
  of multiple problems
*/
template<typename...>
struct bindSingleSolverWithMultipleGalerkinProblems;

template<class solver, class problem>
struct bindSingleSolverWithMultipleGalerkinProblems<solver, std::tuple<problem>>
{
  template<class rom_state_t, class scalar_t, class collector_t>
  static void bind(pybind11::module & m)
  {
    m.def("advanceNSteps",
	  &pressio::rom::galerkin::solveNSteps<
	  problem, rom_state_t, scalar_t, collector_t, solver>);
  }

  template<class rom_state_t, class scalar_t>
  static void bind(pybind11::module & m)
  {
    m.def("advanceNSteps",
	  &pressio::rom::galerkin::solveNSteps<
	  problem, rom_state_t, scalar_t, solver>);
  }
};

template<class solver, class head, class ...tails>
struct bindSingleSolverWithMultipleGalerkinProblems<solver, std::tuple<head, tails...>>
{
  template<class rom_state_t, class ...args>
  static void bind(pybind11::module & m)
  {
    bindSingleSolverWithMultipleGalerkinProblems
      <solver, std::tuple<head>>::template bind<rom_state_t, args...>(m);

    bindSingleSolverWithMultipleGalerkinProblems
      <solver, std::tuple<tails...>>::template bind<rom_state_t, args...>(m);
  }
};

}//end namespace pressio4py
#endif
