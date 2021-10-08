/*
//@HEADER
// ************************************************************************
//
// types.hpp
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

#ifndef PRESSIO4PY_PYBINDINGS_STEADY_LSPG_HPP_
#define PRESSIO4PY_PYBINDINGS_STEADY_LSPG_HPP_

namespace pressio4py{

struct SteadyLSPGBinder
{
  using masker_wrapper_type    = ::pressio4py::MaskerWrapper;
  using precond_wrapper_t = pressio4py::PreconditionerWrapper;

  // default or hypred are same
  using default_problem_t =
    decltype
    (
     pressio::rom::lspg::create_default_steady_problem
     (
      std::declval<rom_steady_adapter_wrapper_type>(),
      std::declval<::pressio4py::decoder_t &>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<::pressio4py::py_f_arr>()
      )
     );
  using default_system_t = typename default_problem_t::traits::steady_system_type;

  // default, prec
  using prec_default_problem_t =
    decltype
    (
     pressio::rom::lspg::create_default_steady_problem
     (
      std::declval<rom_steady_adapter_wrapper_type>(),
      std::declval<::pressio4py::decoder_t &>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<precond_wrapper_t>()
      )
     );
  using prec_default_system_t = typename prec_default_problem_t::traits::steady_system_type;

  // masked
  using masked_problem_t =
    decltype
    (
     pressio::rom::lspg::create_masked_steady_problem
     (
      std::declval<rom_steady_adapter_wrapper_type>(),
      std::declval<::pressio4py::decoder_t &>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<masker_wrapper_type>()
      )
     );
  using masked_system_t = typename masked_problem_t::traits::steady_system_type;

  // masked, prec
  using prec_masked_problem_t =
    decltype
    (
     pressio::rom::lspg::create_masked_steady_problem
     (
      std::declval<rom_steady_adapter_wrapper_type>(),
      std::declval<::pressio4py::decoder_t &>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<masker_wrapper_type>(),
      std::declval<precond_wrapper_t>()
      )
     );
  using prec_masked_system_t = typename prec_masked_problem_t::traits::steady_system_type;


  // collect all problem types in tuples
  using problem_types = std::tuple<
    default_problem_t, prec_default_problem_t,
    masked_problem_t,  prec_masked_problem_t>;

  // collect all system types in tuples
  using system_types = std::tuple<
    default_system_t, prec_default_system_t,
    masked_system_t,  prec_masked_system_t>;

  template<typename T>
  static void bindConstructor(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pybind11::object,
		::pressio4py::decoder_t &,
		::pressio4py::py_f_arr,
		::pressio4py::py_f_arr
		>());
  }

  template<typename T>
  static void bindConstructorPrec(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pybind11::object,
		::pressio4py::decoder_t &,
		::pressio4py::py_f_arr,
		::pressio4py::py_f_arr,
		pybind11::object // prec
		>());
  }

  template<typename T>
  static void bindConstructorMasked(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pybind11::object,
		::pressio4py::decoder_t &,
		::pressio4py::py_f_arr,
		::pressio4py::py_f_arr,
		pybind11::object  // masker
		>());
  }

  template<typename T>
  static void bindConstructorMaskedPrec(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pybind11::object,
		::pressio4py::decoder_t &,
		::pressio4py::py_f_arr,
		::pressio4py::py_f_arr,
		pybind11::object,  // masker
		pybind11::object // prec
		>());
  }

  template<class T>
  static void bindMethods(pybind11::class_<T> & problem)
  {
    problem.def("system",
		&T::system,
		pybind11::return_value_policy::reference);

    problem.def("fomStateReconstructor",
		&T::fomStateReconstructor,
		pybind11::return_value_policy::reference);

    problem.def("createResidual",
		&T::createResidual,
		pybind11::return_value_policy::take_ownership);
    problem.def("createJacobian",
		&T::createJacobian,
		pybind11::return_value_policy::take_ownership);
    problem.def("residual", &T::residual);
    problem.def("jacobian", &T::jacobian);
  }

  static void bindProblems(pybind11::module & m)
  {
    pybind11::class_<default_problem_t> DefProbClass(m, "Problem");
    bindConstructor(DefProbClass);
    bindMethods(DefProbClass);

    pybind11::class_<prec_default_problem_t> PrecDefProbClass(m, "PrecProblem");
    bindConstructorPrec(PrecDefProbClass);
    bindMethods(PrecDefProbClass);

    pybind11::class_<masked_problem_t> MaskedProbClass(m, "MaskedProblem");
    bindConstructorMasked(MaskedProbClass);
    bindMethods(MaskedProbClass);

    pybind11::class_<prec_masked_problem_t> PrecMaskedProbClass(m, "PrecMaskedProblem");
    bindConstructorMaskedPrec(PrecMaskedProbClass);
    bindMethods(PrecMaskedProbClass);
  }
};

}//end namespace pressio4py
#endif
