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

#ifndef PRESSIO4PY_PYBINDINGS_UNSTEADY_LSPG_HPP_
#define PRESSIO4PY_PYBINDINGS_UNSTEADY_LSPG_HPP_

namespace pressio4py{

struct UnsteadyLSPGBinder
{
  using masker_wrapper_type  = ::pressio4py::MaskerWrapper;
  using precond_wrapper_type = ::pressio4py::PreconditionerWrapper;
  using hypred_updater_type  = HypRedUpdaterPressio4py<::pressio4py::scalar_t>;

  //*****************
  // cont-time time
  //*****************
  // default, cont-time
  using default_problem_t =
    decltype
    (
     pressio::rom::lspg::create_default_unsteady_problem
     (
      std::declval<::pressio::ode::StepScheme>(),
      std::declval<rom_conttime_adapter_wrapper_type>(),
      std::declval<::pressio4py::decoder_t &>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<::pressio4py::py_f_arr>()
      )
     );
  using default_stepper_t = typename default_problem_t::traits::stepper_type;

  // default, prec, cont-time
  using prec_default_problem_t =
    decltype
    (
     pressio::rom::lspg::create_default_unsteady_problem
     (
      std::declval<::pressio::ode::StepScheme>(),
      std::declval<rom_conttime_adapter_wrapper_type>(),
      std::declval<::pressio4py::decoder_t &>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<precond_wrapper_type>()
      )
     );
  using prec_default_stepper_t = typename prec_default_problem_t::traits::stepper_type;

  // masked, cont-time
  using masked_problem_t =
    decltype
    (
     pressio::rom::lspg::create_masked_unsteady_problem
     (
      std::declval<::pressio::ode::StepScheme>(),
      std::declval<rom_conttime_adapter_wrapper_type>(),
      std::declval<::pressio4py::decoder_t &>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<masker_wrapper_type>()
      )
     );
  using masked_stepper_t = typename masked_problem_t::traits::stepper_type;

  // masked, prec, cont-time
  using prec_masked_problem_t =
    decltype
    (
     pressio::rom::lspg::create_masked_unsteady_problem
     (
      std::declval<::pressio::ode::StepScheme>(),
      std::declval<rom_conttime_adapter_wrapper_type>(),
      std::declval<::pressio4py::decoder_t &>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<masker_wrapper_type>(),
      std::declval<precond_wrapper_type>()
      )
     );
  using prec_masked_stepper_t = typename prec_masked_problem_t::traits::stepper_type;

  // hypred, cont-time
  using hypred_problem_t =
    decltype
    (
     pressio::rom::lspg::create_hyperreduced_unsteady_problem
     (
      std::declval<::pressio::ode::StepScheme>(),
      std::declval<rom_conttime_adapter_wrapper_type>(),
      std::declval<::pressio4py::decoder_t &>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<hypred_updater_type>()
      )
     );
  using hypred_stepper_t = typename hypred_problem_t::traits::stepper_type;

  // hypred, cont-time, prec
  using prec_hypred_problem_t =
    decltype
    (
     pressio::rom::lspg::create_prec_hyperreduced_unsteady_problem
     (
      std::declval<::pressio::ode::StepScheme>(),
      std::declval<rom_conttime_adapter_wrapper_type>(),
      std::declval<::pressio4py::decoder_t &>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<hypred_updater_type>(),
      std::declval<precond_wrapper_type>()
      )
     );
  using prec_hypred_stepper_t = typename prec_hypred_problem_t::traits::stepper_type;

  //***********************
  // discrete time 2 states
  //***********************
  // default, discrete-time 2
  using default_dt_n2_problem_t =
    decltype
    (
     pressio::rom::lspg::create_default_unsteady_problem<2>
     (
      std::declval<rom_disctime_n2_adapter_wrapper_type>(),
      std::declval<::pressio4py::decoder_t &>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<::pressio4py::py_f_arr>()
      )
     );
  using default_dt_n2_stepper_t = typename default_dt_n2_problem_t::traits::stepper_type;

  // masked, discrete-time 2
  using masked_dt_n2_problem_t =
    decltype
    (
     pressio::rom::lspg::create_masked_unsteady_problem<2>
     (
      std::declval<rom_disctime_n2_adapter_wrapper_type>(),
      std::declval<::pressio4py::decoder_t &>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<masker_wrapper_type>()
      )
     );
  using masked_dt_n2_stepper_t = typename masked_dt_n2_problem_t::traits::stepper_type;

  //***********************
  // discrete time 3 states
  //***********************
  // default, discrete-time 3
  using default_dt_n3_problem_t =
    decltype
    (
     pressio::rom::lspg::create_default_unsteady_problem<3>
     (
      std::declval<rom_disctime_n3_adapter_wrapper_type>(),
      std::declval<::pressio4py::decoder_t &>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<::pressio4py::py_f_arr>()
      )
     );
  using default_dt_n3_stepper_t = typename default_dt_n3_problem_t::traits::stepper_type;

  // masked, discrete-time 3
  using masked_dt_n3_problem_t =
    decltype
    (
     pressio::rom::lspg::create_masked_unsteady_problem<3>
     (
      std::declval<rom_disctime_n3_adapter_wrapper_type>(),
      std::declval<::pressio4py::decoder_t &>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<::pressio4py::py_f_arr>(),
      std::declval<masker_wrapper_type>()
      )
     );
  using masked_dt_n3_stepper_t = typename masked_dt_n3_problem_t::traits::stepper_type;

  // ======================================================
  // collect all types
  // ======================================================

  // collect all problem types in tuples
  using problem_types = std::tuple<
    default_problem_t, prec_default_problem_t,
    masked_problem_t,  prec_masked_problem_t,
    hypred_problem_t,  prec_hypred_problem_t,
    default_dt_n2_problem_t, masked_dt_n2_problem_t,
    default_dt_n3_problem_t, masked_dt_n3_problem_t>;

  // // collect all stepper types in tuples
  // using stepper_types = std::tuple<
  //   default_stepper_t, prec_default_stepper_t,
  //   masked_stepper_t,  prec_masked_stepper_t,
  //   hypred_stepper_t,  prec_hypred_stepper_t,
  //   default_dt_n2_stepper_t, masked_dt_n2_stepper_t,
  //   default_dt_n3_stepper_t, masked_dt_n3_stepper_t>;

  // ======================================================
  // functions
  // ======================================================

  // cont-time
  template<typename T>
  static void bindConstructorDefault(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pressio::ode::StepScheme,
		pybind11::object,
		::pressio4py::decoder_t &,
		::pressio4py::py_f_arr, // rom_state
		::pressio4py::py_f_arr  // fom_state
		>());
  }

  template<typename T>
  static void bindConstructorDefaultPrec(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pressio::ode::StepScheme,
		pybind11::object,
		::pressio4py::decoder_t &,
		::pressio4py::py_f_arr, // rom_state
		::pressio4py::py_f_arr,  // fom_state
		pybind11::object // prec
		>());
  }

  template<typename T>
  static void bindConstructorMasked(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pressio::ode::StepScheme,
		pybind11::object,
		::pressio4py::decoder_t &,
		::pressio4py::py_f_arr, // rom_state
		::pressio4py::py_f_arr,  // fom_state
		pybind11::object  // masker
		>());
  }

  template<typename T>
  static void bindConstructorMaskedPrec(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pressio::ode::StepScheme,
		pybind11::object,
		::pressio4py::decoder_t &,
		::pressio4py::py_f_arr, // rom_state
		::pressio4py::py_f_arr,  // fom_state
		pybind11::object,  // masker
		pybind11::object // prec
		>());
  }

  template<typename T>
  static void bindConstructorHypred(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pressio::ode::StepScheme,
		pybind11::object,
		::pressio4py::decoder_t &,
		::pressio4py::py_f_arr, // rom_state
		::pressio4py::py_f_arr, // fom_state
		hypred_updater_type
		>());
  }

  template<typename T>
  static void bindConstructorHypredPrec(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pressio::ode::StepScheme,
		pybind11::object,
		::pressio4py::decoder_t &,
		::pressio4py::py_f_arr, // rom_state
		::pressio4py::py_f_arr, // fom_state
		hypred_updater_type,
		pybind11::object // prec
		>());
  }

  // discrete-time
  template<typename T>
  static void bindConstructorDefaultDT(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pybind11::object,
		::pressio4py::decoder_t &,
		::pressio4py::py_f_arr, // rom_state
		::pressio4py::py_f_arr  // fom_state
		>());
  }

  template<typename T>
  static void bindConstructorMaskedDT(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pybind11::object,
		::pressio4py::decoder_t &,
		::pressio4py::py_f_arr, // rom_state
		::pressio4py::py_f_arr,  // fom_state
		pybind11::object  // masker
		>());
  }

  template<class T>
  static void bindMethods(pybind11::class_<T> & object)
  {
    // object.def("stepper",
    // 		&T::stepper,
    // 		pybind11::return_value_policy::reference);

    object.def("fomStateReconstructor",
		&T::fomStateReconstructor,
		pybind11::return_value_policy::reference);
  }

  template<
    class SolverType, class T,
    pressio::mpl::enable_if_t<!::pressio4py::is_solver_wrapper<SolverType>::value, int> = 0
    >
  static void bindStepperCallOperator(pybind11::class_<T> & object)
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
  static void bindStepperCallOperator(pybind11::class_<T> & object)
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
  static void bindStepperCallOperatorVar(pybind11::class_<T> & object)
  {
    bindStepperCallOperator<S1, T>(object);
    bindStepperCallOperator<S2, T>(object);
  }

  template<class T>
  static void bindStepperOperatorMethods(pybind11::class_<T> & object)
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

  template<std::size_t n, class T>
  static void bindStepperOperatorMethods2(pybind11::class_<T> & object)
  {
    object.def("createResidual",
		&T::createResidual,
		pybind11::return_value_policy::take_ownership);
    object.def("createJacobian",
		&T::createJacobian,
		pybind11::return_value_policy::take_ownership);

    object.def("residual", &T::template residual<n>);
    object.def("jacobian", &T::template jacobian<n>);
  }

  template<class ...Solvers>
  static void bindProblems(pybind11::module & m)
  {
    pybind11::class_<hypred_updater_type> HypredUpdaterClass(m, "StencilToSampleIndexing");
    HypredUpdaterClass.def(pybind11::init<const std::vector<int> &>());

    // ---- discrete-time 2 states ----
    constexpr auto n = default_dt_n2_stepper_t::numAuxStates;
    pybind11::class_<default_dt_n2_problem_t> DefProbClassDTN2(m, "DiscreteTimeProblemTwoStates");
    bindConstructorDefaultDT		  (DefProbClassDTN2);
    bindMethods				  (DefProbClassDTN2);
    bindStepperCallOperatorVar<Solvers...>(DefProbClassDTN2);
    bindStepperOperatorMethods		  (DefProbClassDTN2);
    pybind11::class_<default_dt_n2_stepper_t> DefStepperClassDTN2(m, "DefaultStepperDTN2");
    bindStepperCallOperatorVar<Solvers...>(DefStepperClassDTN2);
    bindStepperOperatorMethods2<n>	  (DefStepperClassDTN2);

    pybind11::class_<masked_dt_n2_problem_t> MaskedProbClassDTN2(m, "DiscreteTimeMaskedProblemTwoStates");
    bindConstructorMaskedDT		  (MaskedProbClassDTN2);
    bindMethods				  (MaskedProbClassDTN2);
    bindStepperCallOperatorVar<Solvers...>(MaskedProbClassDTN2);
    bindStepperOperatorMethods		  (MaskedProbClassDTN2);
    pybind11::class_<masked_dt_n2_stepper_t> MaskedStepperClassDTN2(m, "MaskedStepperDTN2");
    bindStepperCallOperatorVar<Solvers...>(MaskedStepperClassDTN2);
    bindStepperOperatorMethods2<n>	  (MaskedStepperClassDTN2);

    // ---- discrete-time 3 states ----
    constexpr auto k = default_dt_n3_stepper_t::numAuxStates;
    pybind11::class_<default_dt_n3_problem_t> DefProbClassDTN3(m, "DiscreteTimeProblemThreeStates");
    bindConstructorDefaultDT		  (DefProbClassDTN3);
    bindMethods				  (DefProbClassDTN3);
    bindStepperCallOperatorVar<Solvers...>(DefProbClassDTN3);
    bindStepperOperatorMethods		  (DefProbClassDTN3);
    pybind11::class_<default_dt_n3_stepper_t> DefStepperClassDTN3(m, "DefaultStepperDTN3");
    bindStepperCallOperatorVar<Solvers...>(DefStepperClassDTN3);
    bindStepperOperatorMethods2<k>	  (DefStepperClassDTN3);

    pybind11::class_<masked_dt_n3_problem_t> MaskedProbClassDTN3(m, "DiscreteTimeMaskedProblemThreeStates");
    bindConstructorMaskedDT		  (MaskedProbClassDTN3);
    bindMethods				  (MaskedProbClassDTN3);
    bindStepperCallOperatorVar<Solvers...>(MaskedProbClassDTN3);
    bindStepperOperatorMethods		  (MaskedProbClassDTN3);
    pybind11::class_<masked_dt_n3_stepper_t> MaskedStepperClassDTN3(m, "MaskedStepperDTN3");
    bindStepperCallOperatorVar<Solvers...>(MaskedStepperClassDTN3);
    bindStepperOperatorMethods2<k>	  (MaskedStepperClassDTN3);

    // ----- cont-time ----
    pybind11::class_<default_problem_t> DefProbClass(m, "DefaultProblem");
    bindConstructorDefault		  (DefProbClass);
    bindMethods				  (DefProbClass);
    bindStepperCallOperatorVar<Solvers...>(DefProbClass);
    bindStepperOperatorMethods		  (DefProbClass);
    pybind11::class_<default_stepper_t> DefStepperClass(m, "DefaultStepper");
    bindStepperCallOperatorVar<Solvers...>(DefStepperClass);
    bindStepperOperatorMethods		  (DefStepperClass);

    pybind11::class_<prec_default_problem_t> PrecDefProbClass(m, "PrecDefaultProblem");
    bindConstructorDefaultPrec		  (PrecDefProbClass);
    bindMethods				  (PrecDefProbClass);
    bindStepperCallOperatorVar<Solvers...>(PrecDefProbClass);
    bindStepperOperatorMethods		  (PrecDefProbClass);
    pybind11::class_<prec_default_stepper_t> PrecDefStepperClass(m, "PrecDefaultStepper");
    bindStepperCallOperatorVar<Solvers...>(PrecDefStepperClass);
    bindStepperOperatorMethods		  (PrecDefStepperClass);

    pybind11::class_<masked_problem_t> MaskedProbClass(m, "MaskedProblem");
    bindConstructorMasked		  (MaskedProbClass);
    bindMethods				  (MaskedProbClass);
    bindStepperCallOperatorVar<Solvers...>(MaskedProbClass);
    bindStepperOperatorMethods		  (MaskedProbClass);
    pybind11::class_<masked_stepper_t> MaskedStepperClass(m, "MaskedStepper");
    bindStepperCallOperatorVar<Solvers...>(MaskedStepperClass);
    bindStepperOperatorMethods		  (MaskedStepperClass);

    pybind11::class_<prec_masked_problem_t> PrecMaskedProbClass(m, "PrecMaskedProblem");
    bindConstructorMaskedPrec		  (PrecMaskedProbClass);
    bindMethods				  (PrecMaskedProbClass);
    bindStepperCallOperatorVar<Solvers...>(PrecMaskedProbClass);
    bindStepperOperatorMethods		  (PrecMaskedProbClass);
    pybind11::class_<prec_masked_stepper_t> PrecMaskedStepperClass(m, "PrecMaskedStepper");
    bindStepperCallOperatorVar<Solvers...>(PrecMaskedStepperClass);
    bindStepperOperatorMethods		  (PrecMaskedStepperClass);

    pybind11::class_<hypred_problem_t> HypredProbClass(m, "HypredProblem");
    bindConstructorHypred		  (HypredProbClass);
    bindMethods				  (HypredProbClass);
    bindStepperCallOperatorVar<Solvers...>(HypredProbClass);
    bindStepperOperatorMethods		  (HypredProbClass);
    pybind11::class_<hypred_stepper_t> HypredStepperClass(m, "HypredStepper");
    bindStepperCallOperatorVar<Solvers...>(HypredStepperClass);
    bindStepperOperatorMethods		  (HypredStepperClass);

    pybind11::class_<prec_hypred_problem_t> PrecHypredProbClass(m, "PrecHypredProblem");
    bindConstructorHypredPrec		  (PrecHypredProbClass);
    bindMethods				  (PrecHypredProbClass);
    bindStepperCallOperatorVar<Solvers...>(PrecHypredProbClass);
    bindStepperOperatorMethods		  (PrecHypredProbClass);
    pybind11::class_<prec_hypred_stepper_t> PrecHypredStepperClass(m, "PrecHypredStepper");
    bindStepperCallOperatorVar<Solvers...>(PrecHypredStepperClass);
    bindStepperOperatorMethods		  (PrecHypredStepperClass);
  }

  template<class ...Solvers>
  static void bindSteppers(pybind11::module & m)
  {

    // // ---- disc-time 2 states -----
    // pybind11::class_<default_dt_n2_stepper_t> DefStepperClassDTN2(m, "DefaultStepperDTN2");
    // bindStepperCallOperatorVar<Solvers...>(DefStepperClassDTN2);
    // bindStepperOperatorMethods2(DefStepperClassDTN2);

    // pybind11::class_<masked_dt_n2_stepper_t> MaskedStepperClassDTN2(m, "MaskedStepperDTN2");
    // bindStepperCallOperatorVar<Solvers...>(MaskedStepperClassDTN2);
    // bindStepperOperatorMethods2(MaskedStepperClassDTN2);

    // // ---- disc-time 3 states -----
    // pybind11::class_<default_dt_n3_stepper_t> DefStepperClassDTN3(m, "DefaultStepperDTN3");
    // bindStepperCallOperatorVar<Solvers...>(DefStepperClassDTN3);
    // bindStepperOperatorMethods2(DefStepperClassDTN3);

    // pybind11::class_<masked_dt_n3_stepper_t> MaskedStepperClassDTN3(m, "MaskedStepperDTN3");
    // bindStepperCallOperatorVar<Solvers...>(MaskedStepperClassDTN3);
    // bindStepperOperatorMethods2(MaskedStepperClassDTN3);

    // // ---- cont-time -----
    // pybind11::class_<default_stepper_t> DefStepperClass(m, "DefaultStepper");
    // bindStepperCallOperatorVar<Solvers...>(DefStepperClass);
    // bindStepperOperatorMethods(DefStepperClass);

    // pybind11::class_<prec_default_stepper_t> PrecDefStepperClass(m, "PrecDefaultStepper");
    // bindStepperCallOperatorVar<Solvers...>(PrecDefStepperClass);
    // bindStepperOperatorMethods(PrecDefStepperClass);

    // pybind11::class_<masked_stepper_t> MaskedStepperClass(m, "MaskedStepper");
    // bindStepperCallOperatorVar<Solvers...>(MaskedStepperClass);
    // bindStepperOperatorMethods(MaskedStepperClass);

    // pybind11::class_<prec_masked_stepper_t> PrecMaskedStepperClass(m, "PrecMaskedStepper");
    // bindStepperCallOperatorVar<Solvers...>(PrecMaskedStepperClass);
    // bindStepperOperatorMethods(PrecMaskedStepperClass);

    // pybind11::class_<hypred_stepper_t> HypredStepperClass(m, "HypredStepper");
    // bindStepperCallOperatorVar<Solvers...>(HypredStepperClass);
    // bindStepperOperatorMethods(HypredStepperClass);

    // pybind11::class_<prec_hypred_stepper_t> PrecHypredStepperClass(m, "PrecHypredStepper");
    // bindStepperCallOperatorVar<Solvers...>(PrecHypredStepperClass);
    // bindStepperOperatorMethods(PrecHypredStepperClass);
  }
};

}//end namespace pressio4py
#endif
