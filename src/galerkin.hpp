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

#ifndef PRESSIO4PY_PYBINDINGS_GALERKIN_HPP_
#define PRESSIO4PY_PYBINDINGS_GALERKIN_HPP_

namespace pressio4py{

struct GalerkinBinder
{
  using projector_wrapper_type = ::pressio4py::GalerkinProjWrapper;
  using masker_wrapper_type    = ::pressio4py::MaskerWrapper;

  // explicit default
  using explicit_default_problem_t =
    decltype
    (
     pressio::rom::galerkin::create_default_explicit_problem
     (
      std::declval<::pressio::ode::StepScheme>(),
      std::declval<rom_conttime_adapter_wrapper_type>(),
      std::declval<::pressio4py::decoder_t &>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<::pressio4py::py_f_arr>()
      )
     );
  using explicit_default_stepper_t = typename explicit_default_problem_t::traits::stepper_type;

  // explicit masked
  using explicit_masked_problem_t =
    decltype
    (
     pressio::rom::galerkin::create_masked_explicit_problem
     (
      std::declval<::pressio::ode::StepScheme>(),
      std::declval<rom_conttime_adapter_wrapper_type>(),
      std::declval<::pressio4py::decoder_t &>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<projector_wrapper_type>(),
      std::declval<masker_wrapper_type>()
      )
     );
  using explicit_masked_stepper_t = typename explicit_masked_problem_t::traits::stepper_type;

  // explicit hyper-reduced
  using explicit_hypred_problem_t =
    decltype
    (
     pressio::rom::galerkin::create_hyperreduced_explicit_problem
     (
      std::declval<::pressio::ode::StepScheme>(),
      std::declval<rom_conttime_adapter_wrapper_type>(),
      std::declval<::pressio4py::decoder_t &>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<projector_wrapper_type>()
      )
     );
  using explicit_hypred_stepper_t = typename explicit_hypred_problem_t::traits::stepper_type;

  // implicit default
  using implicit_default_problem_t =
    decltype
    (
     pressio::rom::galerkin::create_default_implicit_problem
     (
      std::declval<::pressio::ode::StepScheme>(),
      std::declval<rom_conttime_adapter_wrapper_type>(),
      std::declval<::pressio4py::decoder_t &>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<::pressio4py::py_f_arr>()
      )
     );
  using implicit_default_stepper_t = typename implicit_default_problem_t::traits::stepper_type;

  // implicit masked
  using implicit_masked_problem_t =
    decltype
    (
     pressio::rom::galerkin::create_masked_implicit_problem
     (
      std::declval<::pressio::ode::StepScheme>(),
      std::declval<rom_conttime_adapter_wrapper_type>(),
      std::declval<::pressio4py::decoder_t &>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<projector_wrapper_type>(),
      std::declval<masker_wrapper_type>()
      )
     );
  using implicit_masked_stepper_t = typename implicit_masked_problem_t::traits::stepper_type;

  // implicit hyper-reduced
  using implicit_hypred_problem_t =
    decltype
    (
     pressio::rom::galerkin::create_hyperreduced_implicit_problem
     (
      std::declval<::pressio::ode::StepScheme>(),
      std::declval<rom_conttime_adapter_wrapper_type>(),
      std::declval<::pressio4py::decoder_t &>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<projector_wrapper_type>()
      )
     );
  using implicit_hypred_stepper_t = typename implicit_hypred_problem_t::traits::stepper_type;

  // collect all problem types in tuples
  using explicit_problem_types = std::tuple<
    explicit_default_problem_t, explicit_masked_problem_t, explicit_hypred_problem_t>;
  using implicit_problem_types = std::tuple<
    implicit_default_problem_t, implicit_masked_problem_t, implicit_hypred_problem_t>;

  // // collect all stepper types in tuples
  // using explicit_stepper_types = std::tuple<
  //   explicit_default_stepper_t, explicit_masked_stepper_t, explicit_hypred_stepper_t>;
  // using implicit_stepper_types = std::tuple<
  //   implicit_default_stepper_t, implicit_masked_stepper_t, implicit_hypred_stepper_t>;

  template<typename T>
  static void bindConstructorDefault(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pressio::ode::StepScheme,
		pybind11::object,
		::pressio4py::decoder_t &,
		::pressio4py::py_f_arr,
		::pressio4py::py_f_arr
		>());
  }

  template<typename T>
  static void bindConstructorMasked(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pressio::ode::StepScheme,
		pybind11::object,
		::pressio4py::decoder_t &,
		::pressio4py::py_f_arr,
		::pressio4py::py_f_arr,
		pybind11::object, // projector
		pybind11::object  // masker
		>());
  }

  template<typename T>
  static void bindConstructorHypRed(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pressio::ode::StepScheme,
		pybind11::object,
		::pressio4py::decoder_t &,
		::pressio4py::py_f_arr,
		::pressio4py::py_f_arr,
		pybind11::object // projector
		>());
  }

  template<class T>
  static void bindCommonMethods(pybind11::class_<T> & object)
  {
    // object.def("stepper",
    // 		&T::stepper,
    // 		pybind11::return_value_policy::reference);

    object.def("fomStateReconstructor",
		&T::fomStateReconstructor,
		pybind11::return_value_policy::reference);
  }

  template<class T>
  static void bindExplicitStepperCallOperator(pybind11::class_<T> & object)
  {
    object.def("__call__",
		[](T & object,
		   ::pressio4py::py_f_arr & state,
		   ::pressio4py::scalar_t time,
		   ::pressio4py::scalar_t dt,
		   int32_t step)
		{
		  object(state, time, dt, step);
		}, pybind11::is_operator());
  }

  template<
    class SolverType, class T,
    pressio::mpl::enable_if_t<!::pressio4py::is_solver_wrapper<SolverType>::value, int> = 0
    >
  static void bindImplicitStepperCallOperator(pybind11::class_<T> & object)
  {
    object.def("__call__",
		[](T & object,
		   ::pressio4py::py_f_arr & state,
		   ::pressio4py::scalar_t time,
		   ::pressio4py::scalar_t dt,
		   int32_t step,
		   SolverType & solver
		   )
		{
		  object(state, time, dt, step, solver);
		}, pybind11::is_operator());
  }

  template<
    class SolverType, class T,
    pressio::mpl::enable_if_t<::pressio4py::is_solver_wrapper<SolverType>::value, int> = 0
    >
  static void bindImplicitStepperCallOperator(pybind11::class_<T> & object)
  {
    object.def("__call__",
		[](T & object,
		   ::pressio4py::py_f_arr & state,
		   ::pressio4py::scalar_t time,
		   ::pressio4py::scalar_t dt,
		   int32_t step,
		   pybind11::object pysolver
		   )
		{
		  UserDefinedNonLinSolverWrapper nlsw(pysolver);
		  object(state, time, dt, step, nlsw);
		}, pybind11::is_operator());
  }

  template<class S1, class S2, class T>
  static void bindImplicitStepperCallOperatorVar(pybind11::class_<T> & object){
    bindImplicitStepperCallOperator<S1, T>(object);
    bindImplicitStepperCallOperator<S2, T>(object);
  }

  template<class T>
  static void bindImplicitStepperOperatorMethods(pybind11::class_<T> & object)
  {
    object.def("createResidual",
		&T::createResidual,
		pybind11::return_value_policy::take_ownership);
    object.def("createJacobian",
		&T::createJacobian,
		pybind11::return_value_policy::take_ownership);
    object.def("residual", &T::residual);
    object.def("jacobian", &T::jacobian);
  }

  static void bindExplicitProblems(pybind11::module & m)
  {
    pybind11::class_<explicit_default_problem_t> DefProbClass(m, "DefaultExplicitProblem");
    bindConstructorDefault(DefProbClass);
    bindCommonMethods(DefProbClass);
    bindExplicitStepperCallOperator(DefProbClass);

    pybind11::class_<explicit_masked_problem_t> MaskedProbClass(m, "MaskedExplicitProblem");
    bindConstructorMasked(MaskedProbClass);
    bindCommonMethods(MaskedProbClass);
    bindExplicitStepperCallOperator(MaskedProbClass);

    pybind11::class_<explicit_hypred_problem_t> HypredProbClass(m, "HyperreducedExplicitProblem");
    bindConstructorHypRed(HypredProbClass);
    bindCommonMethods(HypredProbClass);
    bindExplicitStepperCallOperator(HypredProbClass);
  }

  template<class ...Solvers>
  static void bindImplicitProblems(pybind11::module & m)
  {
    // note that below we need to bind both problem and underlying steppers
    // becauase of how the operator() is implemented: remember that it calls *this
    // otherwise we get a python error that a stepper cannnot be converted to python object

    pybind11::class_<implicit_default_problem_t> DefProbClass(m, "DefaultImplicitProblem");
    bindConstructorDefault(DefProbClass);
    bindCommonMethods(DefProbClass);
    bindImplicitStepperOperatorMethods(DefProbClass);
    bindImplicitStepperCallOperatorVar<Solvers...>(DefProbClass);
    pybind11::class_<implicit_default_stepper_t> ImpDefStepperClass(m,    "DefaultImplicitStepper");
    bindImplicitStepperOperatorMethods(ImpDefStepperClass);
    bindImplicitStepperCallOperatorVar<Solvers...>(ImpDefStepperClass);

    pybind11::class_<implicit_masked_problem_t> MaskedProbClass(m, "MaskedImplicitProblem");
    bindConstructorMasked(MaskedProbClass);
    bindCommonMethods(MaskedProbClass);
    bindImplicitStepperOperatorMethods(MaskedProbClass);
    bindImplicitStepperCallOperatorVar<Solvers...>(MaskedProbClass);
    pybind11::class_<implicit_masked_stepper_t>  ImpMaskedStepperClass(m, "MaskedImplicitStepper");
    bindImplicitStepperOperatorMethods(ImpMaskedStepperClass);
    bindImplicitStepperCallOperatorVar<Solvers...>(ImpMaskedStepperClass);

    pybind11::class_<implicit_hypred_problem_t> HypredProbClass(m, "HyperreducedImplicitProblem");
    bindConstructorHypRed(HypredProbClass);
    bindCommonMethods(HypredProbClass);
    bindImplicitStepperOperatorMethods(HypredProbClass);
    bindImplicitStepperCallOperatorVar<Solvers...>(HypredProbClass);
    pybind11::class_<implicit_hypred_stepper_t>  ImpHypredStepperClass(m, "HyperreducedImplicitStepper");
    bindImplicitStepperOperatorMethods(ImpHypredStepperClass);
    bindImplicitStepperCallOperatorVar<Solvers...>(ImpHypredStepperClass);
  }

  static void bindSteppers(pybind11::module & m)
  {
    pybind11::class_<explicit_default_stepper_t> ExpDefStepperClass(m,    "DefaultExplicitStepper");
    pybind11::class_<explicit_masked_stepper_t>  ExpMaskedStepperClass(m, "MaskedExplicitStepper");
    pybind11::class_<explicit_hypred_stepper_t>  ExpHypredStepperClass(m, "HyperreducedExplicitStepper");
  }
};

}//end namespace pressio4py
#endif
