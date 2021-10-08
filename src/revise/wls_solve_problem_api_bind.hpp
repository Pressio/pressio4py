/*
//@HEADER
// ************************************************************************
//
// wls_solve_problem_api_bind.hpp
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

#ifndef PRESSIO4PY_PYBINDINGS_WLS_SOLVE_PROBLEM_API_BIND_HPP_
#define PRESSIO4PY_PYBINDINGS_WLS_SOLVE_PROBLEM_API_BIND_HPP_

namespace pressio4py
{

/*
  metafunctions to help exposing api for one problem type but mutliple solvers
*/
template<class ...>
struct bindOneWlsProbToMultipleSolvers;

template<class solver_t>
struct bindOneWlsProbToMultipleSolvers<solver_t>
{
  template<class rom_state_t, class wls_system_t, class ...args>
  static void bind(pybind11::module & m)
  {
    m.def("solveWindowsSequentially",
	  &pressio::rom::wls::solveWindowsSequentially<
	  wls_system_t,
	  rom_state_t,
	  solver_t,
	  ::pressio4py::scalar_t>);
  }
};

template<class solver_t, class ...tail_solvers_t>
struct bindOneWlsProbToMultipleSolvers<solver_t, tail_solvers_t...>
{
  template<class rom_state_t, class wls_system_t, class ...args>
  static void bind(pybind11::module & m)
  {
    bindOneWlsProbToMultipleSolvers
      <solver_t>::template bind<rom_state_t, wls_system_t, args...>(m);
    bindOneWlsProbToMultipleSolvers
      <tail_solvers_t...>::template bind<rom_state_t, wls_system_t, args...>(m);
  }
};


/*
  metafunctions to facilitate binding api for the combinatorial case
  of multiple problems with mutliple solvers
*/
template<typename...>
struct bindWlsSystemsWithMultipleSolvers;

template<class s1, class ...solvers>
struct bindWlsSystemsWithMultipleSolvers<std::tuple<s1>, std::tuple<solvers...>>
{
  template<class rom_state_t, class ...args>
  static void bind(pybind11::module & m)
  {
    bindOneWlsProbToMultipleSolvers
      <solvers...>::template bind<rom_state_t, s1, args...>(m);
  }
};

template<class s1, class ...solvers>
struct bindWlsSystemsWithMultipleSolvers<s1, std::tuple<solvers...>>
{
  template<class rom_state_t, class ...args>
  static void bind(pybind11::module & m)
  {
    bindOneWlsProbToMultipleSolvers
      <solvers...>::template bind<rom_state_t, s1, args...>(m);
  }
};

template<class s1, class ... stails, class ...solvers>
struct bindWlsSystemsWithMultipleSolvers<
  std::tuple<s1, stails...>, std::tuple<solvers...>
  >
{
  template<class rom_state_t, class ...args>
  static void bind(pybind11::module & m)
  {
    bindOneWlsProbToMultipleSolvers
      <solvers...>::template bind<rom_state_t, s1, args...>(m);

    bindWlsSystemsWithMultipleSolvers
      <std::tuple<stails...>, std::tuple<solvers...>>::template bind<rom_state_t,args...>(m);
  }
};

}//end namespace pressio4py
#endif
