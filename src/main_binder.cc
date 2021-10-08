/*
//@HEADER
// ************************************************************************
//
// pressio4py.hpp
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

#ifndef PRESSIO4PY_PYBINDINGS_MAIN_BINDER_HPP_
#define PRESSIO4PY_PYBINDINGS_MAIN_BINDER_HPP_

#include "types.hpp"
#include "pressio/ode_steppers_explicit.hpp"
#include "pressio/ode_steppers_implicit.hpp"
#include "pressio/ode_advancers.hpp"
#include "pressio/rom_decoder.hpp"
#include "pressio/rom_galerkin.hpp"
#include "pressio/rom_lspg.hpp"

#include "./hypred_updater.hpp"
#include "./wrappers/ode_system_wrapper.hpp"
#include "./wrappers/galerkin_projector_wrapper.hpp"
#include "./wrappers/masker_wrapper.hpp"
#include "./wrappers/nonlin_solver_wrapper.hpp"
#include "./wrappers/ode_stepper_wrapper.hpp"
#include "./wrappers/precond_wrapper.hpp"
#include "./logger.hpp"
#include "./rom/decoder.hpp"
#include "./rom/fomreconstructor.hpp"
#include "./nonlinear_solvers.hpp"

namespace pressio4py{

template<class ScalarType, class StateType, class ResidualType, class JacobianType>
class ResJacInterface{

protected:
  pybind11::object pyObj_;

public:
  using scalar_type       = ScalarType;
  using state_type	  = StateType;
  using residual_type	  = ResidualType;
  using jacobian_type	  = JacobianType;

public:
  explicit ResJacInterface(pybind11::object pyObj)
    : pyObj_(pyObj){}

  ResJacInterface() = delete;
  ~ResJacInterface() = default;
  ResJacInterface(const ResJacInterface&) = default;
  ResJacInterface & operator=(const ResJacInterface &) = default;
  // note, we don't declare move constructors because pybind11::object
  // gives troubles so just use copy

public:
  residual_type createResidual() const{
    return pyObj_.attr("createResidual")();
  }

  jacobian_type createJacobian() const{
    return pyObj_.attr("createJacobian")();
  }

  void residual(const state_type & state, residual_type & R) const{
    pyObj_.attr("residual")(state, R);
  }

  void jacobian(const state_type & state, jacobian_type & jac) const{
    pyObj_.attr("jacobian")(state, jac);
  }
};

template<class T, class StateType, class SystemType>
T create_explicit_stepper(pressio::ode::StepScheme name,
			  const StateType & state,
			  pybind11::object o)
{
  if (name == pressio::ode::StepScheme::ForwardEuler){
    return T(pressio::ode::ForwardEuler(), state, SystemType(o));
  }

  else if (name == pressio::ode::StepScheme::RungeKutta4){
    return T(pressio::ode::RungeKutta4(), state, SystemType(o));
  }

  else if (name == pressio::ode::StepScheme::AdamsBashforth2){
    return T(pressio::ode::AdamsBashforth2(), state, SystemType(o));
  }

  else if (name == pressio::ode::StepScheme::SSPRungeKutta3){
    return T(pressio::ode::SSPRungeKutta3(), state, SystemType(o));
  }

  else{
    throw std::runtime_error("create_explicit_stepper: invalid StepScheme enum value");
  }
};

template<class T, class StateType, class SystemType>
T create_implicit_stepper(pressio::ode::StepScheme name,
			  const StateType & state,
			  pybind11::object o)
{

  if (name == pressio::ode::StepScheme::BDF1){
    return T(pressio::ode::BDF1(), state, SystemType(o));
  }

  else if (name == pressio::ode::StepScheme::BDF2){
    return T(pressio::ode::BDF2(), state, SystemType(o));
  }

  else if (name == pressio::ode::StepScheme::CrankNicolson){
    return T(pressio::ode::CrankNicolson(), state, SystemType(o));
  }

  else{
    throw std::runtime_error("create_implicit_stepper: invalid StepScheme enum value");
  }
};

// n_steps, fixed dt
// template<class StepperWrapper, class StateType, class TimeType>
// void advance_n_steps_for_py(pybind11::object pystepper,
// 			    StateType state,
// 			    const TimeType start_time,
// 			    const TimeType time_step_size,
// 			    const int32_t num_steps)
// {
//   StepperWrapper stw(pystepper);
//   pressio::ode::advance_n_steps(stw, state, start_time,
// 				time_step_size, num_steps);
// }

template<class StepperType, class StateType, class TimeType>
void advance_n_steps(StepperType & stepper,
		     StateType & state,
		     const TimeType start_time,
		     const TimeType time_step_size,
		     const ::pressio::ode::step_count_type num_steps)
{
  pressio::ode::advance_n_steps(stepper, state, start_time,
				time_step_size, num_steps);
}

template<class StepperType, class StateType, class TimeType, class SolverType>
void advance_n_steps(StepperType & stepper,
		     StateType & state,
		     const TimeType start_time,
		     const TimeType time_step_size,
		     const ::pressio::ode::step_count_type num_steps,
		     SolverType & solver)
{
  pressio::ode::advance_n_steps(stepper, state, start_time,
				time_step_size, num_steps, solver);
}

template<class StepperType, class StateType, class TimeType, class SolverType>
void advance_n_steps_user_solver(StepperType & stepper,
				 StateType & state,
				 const TimeType start_time,
				 const TimeType time_step_size,
				 const ::pressio::ode::step_count_type num_steps,
				 pybind11::object pySolver)
{
  SolverType sw(pySolver);
  pressio::ode::advance_n_steps(stepper, state, start_time,
				time_step_size, num_steps, sw);
}

// n_steps, arbitrary dt
template<class StepperType, class StateType, class TimeType, class DtSetter>
void advance_n_steps_dtcb(StepperType & stepper,
			  StateType & state,
			  const TimeType start_time,
			  pybind11::object pyStepSetter,
			  const ::pressio::ode::step_count_type num_steps)
{
  const DtSetter dtSetter(pyStepSetter);
  pressio::ode::advance_n_steps(stepper, state, start_time, dtSetter, num_steps);
}

template<
  class StepperType, class StateType, class TimeType,
  class DtSetter, class SolverType
  >
void advance_n_steps_dtcb(StepperType & stepper,
			  StateType & state,
			  const TimeType start_time,
			  pybind11::object pyStepSetter,
			  const ::pressio::ode::step_count_type num_steps,
			  SolverType & solver)
{
  const DtSetter dtSetter(pyStepSetter);
  pressio::ode::advance_n_steps(stepper, state, start_time,
				dtSetter, num_steps, solver);
}

template<
  class StepperType, class StateType, class TimeType,
  class DtSetter, class SolverType
  >
void advance_n_steps_dtcb_user_solver(StepperType & stepper,
				      StateType & state,
				      const TimeType start_time,
				      pybind11::object pyStepSetter,
				      const ::pressio::ode::step_count_type num_steps,
				      pybind11::object pySolver)
{
  SolverType sw(pySolver);
  const DtSetter dtSetter(pyStepSetter);
  pressio::ode::advance_n_steps(stepper, state, start_time,
				dtSetter, num_steps, sw);
}

// observe, fixed dt
template<class StepperType, class StateType, class TimeType, class ObserverType>
void advance_n_steps_and_observe(StepperType & stepper,
				 StateType & state,
				 const TimeType start_time,
				 const TimeType time_step_size,
				 const ::pressio::ode::step_count_type num_steps,
				 pybind11::object pyobserver)
{
  ObserverType obs(pyobserver);
  pressio::ode::advance_n_steps_and_observe(stepper, state, start_time,
					    time_step_size, num_steps, obs);
}

template<
  class StepperType, class StateType, class TimeType,
  class ObserverType, class SolverType
  >
void advance_n_steps_and_observe(StepperType & stepper,
				 StateType & state,
				 const TimeType start_time,
				 const TimeType time_step_size,
				 const ::pressio::ode::step_count_type num_steps,
				 pybind11::object pyobserver,
				 SolverType & solver)
{
  ObserverType obs(pyobserver);
  pressio::ode::advance_n_steps_and_observe(stepper, state, start_time,
					    time_step_size, num_steps,
					    obs, solver);
}

template<
  class StepperType, class StateType, class TimeType,
  class ObserverType, class SolverType
  >
void advance_n_steps_and_observe_user_solver(StepperType & stepper,
					     StateType & state,
					     const TimeType start_time,
					     const TimeType time_step_size,
					     const ::pressio::ode::step_count_type num_steps,
					     pybind11::object pyobserver,
					     pybind11::object pySolver)
{
  SolverType sw(pySolver);
  ObserverType obs(pyobserver);
  pressio::ode::advance_n_steps_and_observe(stepper, state, start_time,
					    time_step_size, num_steps, obs, sw);
}

// observe, dt callback
template<
  class StepperType, class StateType, class TimeType,
  class DtSetter, class ObserverType
  >
void advance_n_steps_and_observe_dtcb(StepperType & stepper,
				      StateType & state,
				      const TimeType start_time,
				      pybind11::object pyStepSetter,
				      const ::pressio::ode::step_count_type num_steps,
				      pybind11::object pyobserver)
{
  const DtSetter dtSetter(pyStepSetter);
  ObserverType obs(pyobserver);
  pressio::ode::advance_n_steps_and_observe(stepper, state, start_time,
					    dtSetter, num_steps, obs);
}

template<
  class StepperType, class StateType,
  class TimeType, class DtSetter,
  class ObserverType, class SolverType
  >
void advance_n_steps_and_observe_dtcb(StepperType & stepper,
				      StateType & state,
				      const TimeType start_time,
				      pybind11::object pyStepSetter,
				      const ::pressio::ode::step_count_type num_steps,
				      pybind11::object pyobserver,
				      SolverType & solver)
{
  const DtSetter dtSetter(pyStepSetter);
  ObserverType obs(pyobserver);
  pressio::ode::advance_n_steps_and_observe(stepper, state, start_time,
					    dtSetter, num_steps,
					    obs, solver);
}

template<
  class StepperType, class StateType,
  class TimeType, class DtSetter,
  class ObserverType, class SolverType
  >
void advance_n_steps_and_observe_dtcb_user_solver(StepperType & stepper,
						  StateType & state,
						  const TimeType start_time,
						  pybind11::object pyStepSetter,
						  const ::pressio::ode::step_count_type num_steps,
						  pybind11::object pyobserver,
						  pybind11::object pySolver)
{
  SolverType sw(pySolver);
  const DtSetter dtSetter(pyStepSetter);
  ObserverType obs(pyobserver);
  pressio::ode::advance_n_steps_and_observe(stepper, state, start_time,
					    dtSetter, num_steps, obs, sw);
}


template<typename...> struct BindAdvanceFunctions;

template<class StepperType>
struct BindAdvanceFunctions<std::tuple<StepperType>>
{

  template<class ... Args>
  static void fixedStepSize(pybind11::module & m)
  {
    m.def("advance_n_steps",
	  &advance_n_steps<
	  StepperType, py_f_arr, ::pressio4py::scalar_t, Args...>);

    m.def("advance_n_steps_and_observe",
	  &advance_n_steps_and_observe<
	  StepperType, py_f_arr, ::pressio4py::scalar_t, ode_observer_wrapper_type, Args...>);
  }

  template<class T>
  static void fixedStepSizeUserSolver(pybind11::module & m)
  {
    m.def("advance_n_steps",
	  &advance_n_steps_user_solver<
	  StepperType, py_f_arr, ::pressio4py::scalar_t, T>);

    m.def("advance_n_steps_and_observe",
	  &advance_n_steps_and_observe_user_solver<
	  StepperType, py_f_arr, ::pressio4py::scalar_t, ode_observer_wrapper_type, T>);
  }

  template<class DtSetter, class ... Args>
  static void arbitraryStepSize(pybind11::module & m)
  {
    m.def("advance_n_steps",
	  &advance_n_steps_dtcb<
	  StepperType, py_f_arr, ::pressio4py::scalar_t, DtSetter, Args...>);

    m.def("advance_n_steps_and_observe",
	  &advance_n_steps_and_observe_dtcb<
	  StepperType, py_f_arr, ::pressio4py::scalar_t, DtSetter, ode_observer_wrapper_type, Args...>);
  }

  template<class DtSetter, class T>
  static void arbitraryStepSizeUserSolver(pybind11::module & m)
  {
    m.def("advance_n_steps",
	  &advance_n_steps_dtcb_user_solver<
	  StepperType, py_f_arr, ::pressio4py::scalar_t, DtSetter, T>);

    m.def("advance_n_steps_and_observe",
	  &advance_n_steps_and_observe_dtcb_user_solver<
	  StepperType, py_f_arr, ::pressio4py::scalar_t, DtSetter, ode_observer_wrapper_type, T>);
  }

};

template<class Head, class ... Tail>
struct BindAdvanceFunctions<std::tuple<Head, Tail...>>
{
  template<class ...Args>
  static void fixedStepSize(pybind11::module & m)
  {
    BindAdvanceFunctions<std::tuple<Head>>::template fixedStepSize<Args...>(m);
    BindAdvanceFunctions<std::tuple<Tail...>>::template fixedStepSize<Args...>(m);
  }

  template<class T>
  static void fixedStepSizeUserSolver(pybind11::module & m)
  {
    BindAdvanceFunctions<std::tuple<Head>>::template fixedStepSizeUserSolver<T>(m);
    BindAdvanceFunctions<std::tuple<Tail...>>::template fixedStepSizeUserSolver<T>(m);
  }

  template<class DtSetter, class ... Args>
  static void arbitraryStepSize(pybind11::module & m)
  {
    BindAdvanceFunctions<std::tuple<Head>>::template arbitraryStepSize<DtSetter, Args...>(m);
    BindAdvanceFunctions<std::tuple<Tail...>>::template arbitraryStepSize<DtSetter, Args...>(m);
  }

  template<class DtSetter, class T>
  static void arbitraryStepSizeUserSolver(pybind11::module & m)
  {
    BindAdvanceFunctions<std::tuple<Head>>::template arbitraryStepSizeUserSolver<DtSetter, T>(m);
    BindAdvanceFunctions<std::tuple<Tail...>>::template arbitraryStepSizeUserSolver<DtSetter, T>(m);
  }
};
// --------------------------------------------------------------------

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

  // collect all stepper types in tuples
  using explicit_stepper_types = std::tuple<
    explicit_default_stepper_t, explicit_masked_stepper_t, explicit_hypred_stepper_t>;
  using implicit_stepper_types = std::tuple<
    implicit_default_stepper_t, implicit_masked_stepper_t, implicit_hypred_stepper_t>;

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
  static void bindMethods(pybind11::class_<T> & problem)
  {
    problem.def("stepper",
		&T::stepper,
		pybind11::return_value_policy::reference);

    problem.def("fomStateReconstructor",
		&T::fomStateReconstructor,
		pybind11::return_value_policy::reference);
  }

  static void bindProblems(pybind11::module & m)
  {
    pybind11::class_<explicit_default_problem_t> DefProbClass(m, "DefaultExplicitProblem");
    bindConstructorDefault(DefProbClass);
    bindMethods(DefProbClass);

    pybind11::class_<explicit_masked_problem_t> MaskedProbClass(m, "MaskedExplicitProblem");
    bindConstructorMasked(MaskedProbClass);
    bindMethods(MaskedProbClass);

    pybind11::class_<explicit_hypred_problem_t> HypredProbClass(m, "HyperreducedExplicitProblem");
    bindConstructorHypRed(HypredProbClass);
    bindMethods(HypredProbClass);

    pybind11::class_<implicit_default_problem_t> ImpDefProbClass(m, "DefaultImplicitProblem");
    bindConstructorDefault(ImpDefProbClass);
    bindMethods(ImpDefProbClass);

    pybind11::class_<implicit_masked_problem_t> ImpMaskedProbClass(m, "MaskedImplicitProblem");
    bindConstructorMasked(ImpMaskedProbClass);
    bindMethods(ImpMaskedProbClass);

    pybind11::class_<implicit_hypred_problem_t> ImpHypredProbClass(m, "HyperreducedImplicitProblem");
    bindConstructorHypRed(ImpHypredProbClass);
    bindMethods(ImpHypredProbClass);
  }

  template<class T>
  static void bindExplicitStepperCallOperator(pybind11::class_<T> & stepper)
  {
    stepper.def("__call__",
		[](T & stepper,
		   ::pressio4py::py_f_arr & state,
		   ::pressio4py::scalar_t time,
		   ::pressio4py::scalar_t dt,
		   int32_t step)
		{
		  stepper(state, time, dt, step);
		}, pybind11::is_operator());
  }

  template<
    class SolverType, class T,
    pressio::mpl::enable_if_t<!::pressio4py::is_solver_wrapper<SolverType>::value, int> = 0
    >
  static void bindImplicitStepperCallOperator(pybind11::class_<T> & stepper)
  {
    stepper.def("__call__",
		[](T & stepper,
		   ::pressio4py::py_f_arr & state,
		   ::pressio4py::scalar_t time,
		   ::pressio4py::scalar_t dt,
		   int32_t step,
		   SolverType & solver
		   )
		{
		  stepper(state, time, dt, step, solver);
		}, pybind11::is_operator());
  }

  template<
    class SolverType, class T,
    pressio::mpl::enable_if_t<::pressio4py::is_solver_wrapper<SolverType>::value, int> = 0
    >
  static void bindImplicitStepperCallOperator(pybind11::class_<T> & stepper)
  {
    stepper.def("__call__",
		[](T & stepper,
		   ::pressio4py::py_f_arr & state,
		   ::pressio4py::scalar_t time,
		   ::pressio4py::scalar_t dt,
		   int32_t step,
		   pybind11::object pysolver
		   )
		{
		  UserDefinedNonLinSolverWrapper nlsw(pysolver);
		  stepper(state, time, dt, step, nlsw);
		}, pybind11::is_operator());
  }

  template<class Head, class ...Tail, class T>
  static void bindImplicitStepperCallOperatorVar(pybind11::class_<T> & stepper)
  {
    bindImplicitStepperCallOperator<Head, T>(stepper);
    bindImplicitStepperCallOperator<Tail..., T>(stepper);
  }

  template<class T>
  static void bindImplicitStepperOperatorMethods(pybind11::class_<T> & stepper)
  {
    stepper.def("createResidual",
		&T::createResidual,
		pybind11::return_value_policy::take_ownership);
    stepper.def("createJacobian",
		&T::createJacobian,
		pybind11::return_value_policy::take_ownership);
    stepper.def("residual", &T::residual);
    stepper.def("jacobian", &T::jacobian);
  }

  static void bindExplicitSteppers(pybind11::module & m)
  {
    pybind11::class_<explicit_default_stepper_t> ExpDefStepperClass(m,    "DefaultExplicitStepper");
    bindExplicitStepperCallOperator(ExpDefStepperClass);

    pybind11::class_<explicit_masked_stepper_t>  ExpMaskedStepperClass(m, "MaskedExplicitStepper");
    bindExplicitStepperCallOperator(ExpMaskedStepperClass);

    pybind11::class_<explicit_hypred_stepper_t>  ExpHypredStepperClass(m, "HyperreducedExplicitStepper");
    bindExplicitStepperCallOperator(ExpHypredStepperClass);
  }

  template<class ...Solvers>
  static void bindImplicitSteppers(pybind11::module & m)
  {
    pybind11::class_<implicit_default_stepper_t> ImpDefStepperClass(m,    "DefaultImplicitStepper");
    bindImplicitStepperCallOperatorVar<Solvers...>(ImpDefStepperClass);
    bindImplicitStepperOperatorMethods(ImpDefStepperClass);

    pybind11::class_<implicit_masked_stepper_t>  ImpMaskedStepperClass(m, "MaskedImplicitStepper");
    bindImplicitStepperCallOperatorVar<Solvers...>(ImpMaskedStepperClass);
    bindImplicitStepperOperatorMethods(ImpMaskedStepperClass);

    pybind11::class_<implicit_hypred_stepper_t>  ImpHypredStepperClass(m, "HyperreducedImplicitStepper");
    bindImplicitStepperCallOperatorVar<Solvers...>(ImpHypredStepperClass);
    bindImplicitStepperOperatorMethods(ImpHypredStepperClass);
  }
};


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

  // collect all stepper types in tuples
  using stepper_types = std::tuple<
    default_stepper_t, prec_default_stepper_t,
    masked_stepper_t,  prec_masked_stepper_t,
    hypred_stepper_t,  prec_hypred_stepper_t,
    default_dt_n2_stepper_t, masked_dt_n2_stepper_t,
    default_dt_n3_stepper_t, masked_dt_n3_stepper_t>;

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
  static void bindMethods(pybind11::class_<T> & problem)
  {
    problem.def("stepper",
		&T::stepper,
		pybind11::return_value_policy::reference);

    problem.def("fomStateReconstructor",
		&T::fomStateReconstructor,
		pybind11::return_value_policy::reference);
  }

  static void bindProblems(pybind11::module & m)
  {
    pybind11::class_<hypred_updater_type> HypredUpdaterClass(m, "StencilToSampleIndexing");
    HypredUpdaterClass.def(pybind11::init<const std::vector<int> &>());

    // ---- discrete-time 2 states ----
    pybind11::class_<default_dt_n2_problem_t> DefProbClassDTN2(m, "ProblemTwoStates");
    bindConstructorDefaultDT(DefProbClassDTN2);
    bindMethods(DefProbClassDTN2);

    pybind11::class_<masked_dt_n2_problem_t> MaskedProbClassDTN2(m, "MaskedProblemTwoStates");
    bindConstructorMaskedDT(MaskedProbClassDTN2);
    bindMethods(MaskedProbClassDTN2);

    // ---- discrete-time 3 states ----
    pybind11::class_<default_dt_n3_problem_t> DefProbClassDTN3(m, "ProblemThreeStates");
    bindConstructorDefaultDT(DefProbClassDTN3);
    bindMethods(DefProbClassDTN3);

    pybind11::class_<masked_dt_n3_problem_t> MaskedProbClassDTN3(m, "MaskedProblemThreeStates");
    bindConstructorMaskedDT(MaskedProbClassDTN3);
    bindMethods(MaskedProbClassDTN3);

    // ----- cont-time ----
    pybind11::class_<default_problem_t> DefProbClass(m, "DefaultProblem");
    bindConstructorDefault(DefProbClass);
    bindMethods(DefProbClass);

    pybind11::class_<prec_default_problem_t> PrecDefProbClass(m, "PrecDefaultProblem");
    bindConstructorDefaultPrec(PrecDefProbClass);
    bindMethods(PrecDefProbClass);

    pybind11::class_<masked_problem_t> MaskedProbClass(m, "MaskedProblem");
    bindConstructorMasked(MaskedProbClass);
    bindMethods(MaskedProbClass);

    pybind11::class_<prec_masked_problem_t> PrecMaskedProbClass(m, "PrecMaskedProblem");
    bindConstructorMaskedPrec(PrecMaskedProbClass);
    bindMethods(PrecMaskedProbClass);

    pybind11::class_<hypred_problem_t> HypredProbClass(m, "HypredProblem");
    bindConstructorHypred(HypredProbClass);
    bindMethods(HypredProbClass);

    pybind11::class_<prec_hypred_problem_t> PrecHypredProbClass(m, "PrecHypredProblem");
    bindConstructorHypredPrec(PrecHypredProbClass);
    bindMethods(PrecHypredProbClass);
  }

  template<
    class SolverType, class T,
    pressio::mpl::enable_if_t<!::pressio4py::is_solver_wrapper<SolverType>::value, int> = 0
    >
  static void bindStepperCallOperator(pybind11::class_<T> & stepper)
  {
    stepper.def("__call__",
		[](T & stepper,
		   ::pressio4py::py_f_arr & state,
		   ::pressio4py::scalar_t time,
		   ::pressio4py::scalar_t dt,
		   int32_t step,
		   SolverType & solver
		   )
		{
		  stepper(state, time, dt, step, solver);
		}, pybind11::is_operator());
  }

  template<
    class SolverType, class T,
    pressio::mpl::enable_if_t<::pressio4py::is_solver_wrapper<SolverType>::value, int> = 0
    >
  static void bindStepperCallOperator(pybind11::class_<T> & stepper)
  {
    stepper.def("__call__",
		[](T & stepper,
		   ::pressio4py::py_f_arr & state,
		   ::pressio4py::scalar_t time,
		   ::pressio4py::scalar_t dt,
		   int32_t step,
		   pybind11::object pysolver
		   )
		{
		  UserDefinedNonLinSolverWrapper nlsw(pysolver);
		  stepper(state, time, dt, step, nlsw);
		}, pybind11::is_operator());
  }

  template<class Head, class ...Tail, class T>
  static void bindStepperCallOperatorVar(pybind11::class_<T> & stepper)
  {
    bindStepperCallOperator<Head, T>(stepper);
    bindStepperCallOperator<Tail..., T>(stepper);
  }

  template<class T>
  static void bindStepperOperatorMethods(pybind11::class_<T> & stepper)
  {
    stepper.def("createResidual",
		&T::createResidual,
		pybind11::return_value_policy::take_ownership);
    stepper.def("createJacobian",
		&T::createJacobian,
		pybind11::return_value_policy::take_ownership);
    stepper.def("residual", &T::residual);
    stepper.def("jacobian", &T::jacobian);
  }

  template<class T>
  static void bindStepperOperatorMethods2(pybind11::class_<T> & stepper)
  {
    stepper.def("createResidual",
		&T::createResidual,
		pybind11::return_value_policy::take_ownership);
    stepper.def("createJacobian",
		&T::createJacobian,
		pybind11::return_value_policy::take_ownership);

    constexpr auto n = T::numAuxStates;
    stepper.def("residual", &T::template residual<n>);
    stepper.def("jacobian", &T::template jacobian<n>);
  }

  template<class ...Solvers>
  static void bindSteppers(pybind11::module & m)
  {

    // ---- disc-time 2 states -----
    pybind11::class_<default_dt_n2_stepper_t> DefStepperClassDTN2(m, "DefaultStepperDTN2");
    bindStepperCallOperatorVar<Solvers...>(DefStepperClassDTN2);
    bindStepperOperatorMethods2(DefStepperClassDTN2);

    pybind11::class_<masked_dt_n2_stepper_t> MaskedStepperClassDTN2(m, "MaskedStepperDTN2");
    bindStepperCallOperatorVar<Solvers...>(MaskedStepperClassDTN2);
    bindStepperOperatorMethods2(MaskedStepperClassDTN2);

    // ---- disc-time 3 states -----
    pybind11::class_<default_dt_n3_stepper_t> DefStepperClassDTN3(m, "DefaultStepperDTN3");
    bindStepperCallOperatorVar<Solvers...>(DefStepperClassDTN3);
    bindStepperOperatorMethods2(DefStepperClassDTN3);

    pybind11::class_<masked_dt_n3_stepper_t> MaskedStepperClassDTN3(m, "MaskedStepperDTN3");
    bindStepperCallOperatorVar<Solvers...>(MaskedStepperClassDTN3);
    bindStepperOperatorMethods2(MaskedStepperClassDTN3);

    // ---- cont-time -----
    pybind11::class_<default_stepper_t> DefStepperClass(m, "DefaultStepper");
    bindStepperCallOperatorVar<Solvers...>(DefStepperClass);
    bindStepperOperatorMethods(DefStepperClass);

    pybind11::class_<prec_default_stepper_t> PrecDefStepperClass(m, "PrecDefaultStepper");
    bindStepperCallOperatorVar<Solvers...>(PrecDefStepperClass);
    bindStepperOperatorMethods(PrecDefStepperClass);

    pybind11::class_<masked_stepper_t> MaskedStepperClass(m, "MaskedStepper");
    bindStepperCallOperatorVar<Solvers...>(MaskedStepperClass);
    bindStepperOperatorMethods(MaskedStepperClass);

    pybind11::class_<prec_masked_stepper_t> PrecMaskedStepperClass(m, "PrecMaskedStepper");
    bindStepperCallOperatorVar<Solvers...>(PrecMaskedStepperClass);
    bindStepperOperatorMethods(PrecMaskedStepperClass);

    pybind11::class_<hypred_stepper_t> HypredStepperClass(m, "HypredStepper");
    bindStepperCallOperatorVar<Solvers...>(HypredStepperClass);
    bindStepperOperatorMethods(HypredStepperClass);

    pybind11::class_<prec_hypred_stepper_t> PrecHypredStepperClass(m, "PrecHypredStepper");
    bindStepperCallOperatorVar<Solvers...>(PrecHypredStepperClass);
    bindStepperOperatorMethods(PrecHypredStepperClass);
  }
};

}// end namespace pressio4py

PYBIND11_MODULE(MODNAME, topLevelModule)
{
  // create all modules with correct hierarchy
  pybind11::module solversModule      = topLevelModule.def_submodule("solvers");
  pybind11::module odeModule	      = topLevelModule.def_submodule("ode");
  pybind11::module romModule	      = topLevelModule.def_submodule("rom");
  pybind11::module galerkinModule     = romModule.def_submodule("galerkin");
  pybind11::module lspgModule         = romModule.def_submodule("lspg");
  pybind11::module steadyLspgModule   = lspgModule.def_submodule("steady");
  pybind11::module unsteadyLspgModule = lspgModule.def_submodule("unsteady");

  pressio4py::bindLogger(topLevelModule);

  // =========================
  // NONLINEAR SOLVERS
  // =========================
  pressio4py::solvers::bindUpdatingEnums(solversModule);
  pressio4py::solvers::bindStoppingEnums(solversModule);

  using rj_system_type = pressio4py::ResJacInterface<
    pressio4py::scalar_t, pressio4py::py_f_arr, pressio4py::py_f_arr, pressio4py::py_f_arr>;

  using newraphbinder_t = pressio4py::solvers::NewtonRaphsonBinder<
    pressio4py::linear_solver_wrapper_t, rj_system_type>;
  using newraph_solver_t = typename newraphbinder_t::nonlinear_solver_t;
  newraphbinder_t::bindClassAndMethods(solversModule);

  using gnbinder_t = pressio4py::solvers::GNNormalEqResJacApiBinder<
    pressio4py::linear_solver_wrapper_t, rj_system_type>;
  using gn_neq_solver_t = typename gnbinder_t::nonlinear_solver_t;
  gnbinder_t::bindClassAndMethods(solversModule);

  using wgnbinder_t = pressio4py::solvers::WeighGNNormalEqResJacApiBinder<
    pressio4py::linear_solver_wrapper_t, rj_system_type, pressio4py::nonlin_ls_weigh_wrapper_t>;
  using wgn_neq_solver_t = typename wgnbinder_t::nonlinear_solver_t;
  wgnbinder_t::bindClassAndMethods(solversModule);

  using gnqrbinder_t = pressio4py::solvers::GNQRResJacApiBinder<
    pressio4py::qr_solver_wrapper_t, rj_system_type>;
  using gn_qr_solver_t = typename gnqrbinder_t::nonlinear_solver_t;
  gnqrbinder_t::bindClassAndMethods(solversModule);

  using lmbinder_t = pressio4py::solvers::LMNormalEqResJacApiBinder<
    pressio4py::linear_solver_wrapper_t, rj_system_type>;
  using lm_neq_solver_t = typename lmbinder_t::nonlinear_solver_t;
  lmbinder_t::bindClassAndMethods(solversModule);

  using wlmbinder_t = pressio4py::solvers::WeighLMNormalEqResJacApiBinder<
    pressio4py::linear_solver_wrapper_t, rj_system_type, pressio4py::nonlin_ls_weigh_wrapper_t>;
  using wlm_neq_solver_t = typename wlmbinder_t::nonlinear_solver_t;
  wlmbinder_t::bindClassAndMethods(solversModule);

  // =========================
  // ODE
  // =========================
  pybind11::enum_<pressio::ode::StepScheme>(odeModule, "stepscheme")
    .value("ForwardEuler",      pressio::ode::StepScheme::ForwardEuler)
    .value("RungeKutta4",	pressio::ode::StepScheme::RungeKutta4)
    .value("AdamsBashforth2",	pressio::ode::StepScheme::AdamsBashforth2)
    .value("SSPRungeKutta3",	pressio::ode::StepScheme::SSPRungeKutta3)
    .value("BDF1",		pressio::ode::StepScheme::BDF1)
    .value("BDF2",		pressio::ode::StepScheme::BDF2)
    .value("CrankNicolson",	pressio::ode::StepScheme::CrankNicolson)
    .value("ImplicitArbitrary",	pressio::ode::StepScheme::ImplicitArbitrary)
    .export_values();

  // explicit
  using ode_explicit_system_wrapper = pressio4py::OdeSystemExplicitWrapper<
    pressio4py::scalar_t, pressio4py::py_f_arr, pressio4py::py_f_arr>;
  using ode_explicit_stepper = pressio::ode::impl::ExplicitStepper<
    pressio4py::scalar_t, pressio4py::py_f_arr, ode_explicit_system_wrapper, pressio4py::py_f_arr>;

  pybind11::class_<ode_explicit_stepper> odeExpStep(odeModule, "ExplicitStepper");
  odeExpStep.def("order", &ode_explicit_stepper::order);
  odeExpStep.def("__call__",
		 [](ode_explicit_stepper & stepper,
		    pressio4py::py_f_arr & state,
		    pressio4py::scalar_t time,
		    pressio4py::scalar_t dt,
		    int32_t step)
		 {
		   stepper(state, time, dt, step);
		 }, pybind11::is_operator());

  odeModule.def("create_explicit_stepper",
		&pressio4py::create_explicit_stepper<
		ode_explicit_stepper, pressio4py::py_f_arr, ode_explicit_system_wrapper>,
		pybind11::return_value_policy::take_ownership);

  pressio4py::BindAdvanceFunctions<std::tuple<ode_explicit_stepper>>::fixedStepSize(odeModule);
  pressio4py::BindAdvanceFunctions<std::tuple<ode_explicit_stepper>>::template arbitraryStepSize<
    pressio4py::ode_dt_setter_wrapper_type>(odeModule);

  // implicit cont-time
  using ode_implicit_system_conttime_wrapper = pressio4py::OdeSystemImplicitContTimeWrapper<
    pressio4py::scalar_t, pressio4py::py_f_arr, pressio4py::py_f_arr, pressio4py::py_f_arr>;
  using ode_implicit_stepper = pressio::ode::impl::ImplicitCompose<
    ode_implicit_system_conttime_wrapper, pressio4py::py_f_arr>::type;

  pybind11::class_<ode_implicit_stepper> odeImpStep(odeModule, "ImplicitStepper");
  odeImpStep.def("order", &ode_implicit_stepper::order);
  odeImpStep.def("createResidual",
		 &ode_implicit_stepper::createResidual,
		 pybind11::return_value_policy::take_ownership);
  odeImpStep.def("createJacobian",
		 &ode_implicit_stepper::createJacobian,
		 pybind11::return_value_policy::take_ownership);
  odeImpStep.def("residual", &ode_implicit_stepper::residual);
  odeImpStep.def("jacobian", &ode_implicit_stepper::jacobian);

  odeImpStep.def("__call__",
		 [](ode_implicit_stepper & stepper,
		    ::pressio4py::py_f_arr & state,
		    ::pressio4py::scalar_t time,
		    ::pressio4py::scalar_t dt,
		    int32_t step,
		    newraph_solver_t & solver
		    )
		 {
		   stepper(state, time, dt, step, solver);
		 }, pybind11::is_operator());

  odeImpStep.def("__call__",
		  [](ode_implicit_stepper & stepper,
		     ::pressio4py::py_f_arr & state,
		     ::pressio4py::scalar_t time,
		     ::pressio4py::scalar_t dt,
		     int32_t step,
		     pybind11::object pysolver
		     )
		  {
		    pressio4py::UserDefinedNonLinSolverWrapper nlsw(pysolver);
		    stepper(state, time, dt, step, nlsw);
		  }, pybind11::is_operator());

  odeModule.def("create_implicit_stepper",
		&pressio4py::create_implicit_stepper<
		ode_implicit_stepper, pressio4py::py_f_arr, ode_implicit_system_conttime_wrapper>,
		pybind11::return_value_policy::take_ownership);

  // // bind constructor specialization of Newt-Raph solver
  // newraphbinder_t::bindCreate<ode_implicit_stepper>(solversModule);

  pressio4py::BindAdvanceFunctions<
    std::tuple<ode_implicit_stepper>>::template fixedStepSize<newraph_solver_t>(odeModule);
  pressio4py::BindAdvanceFunctions<
    std::tuple<ode_implicit_stepper>>::template arbitraryStepSize<
      pressio4py::ode_dt_setter_wrapper_type, newraph_solver_t>(odeModule);

  pressio4py::BindAdvanceFunctions<
    std::tuple<ode_implicit_stepper>>::template fixedStepSizeUserSolver<
      pressio4py::UserDefinedNonLinSolverWrapper>(odeModule);
  pressio4py::BindAdvanceFunctions<
    std::tuple<ode_implicit_stepper>>::template arbitraryStepSizeUserSolver<
      pressio4py::ode_dt_setter_wrapper_type, pressio4py::UserDefinedNonLinSolverWrapper>(odeModule);

  // =========================
  // bind ROM
  // =========================
  pressio4py::rom::bindDecoder(romModule);
  pressio4py::rom::bindFomReconstructor(romModule);

  // *** galerkin ***
  using galerkin_binder = pressio4py::GalerkinBinder;
  using galerkin_explicit_steppers = typename galerkin_binder::explicit_stepper_types;
  using galerkin_implicit_steppers = typename galerkin_binder::implicit_stepper_types;
  galerkin_binder::bindProblems(galerkinModule);
  galerkin_binder::bindExplicitSteppers(galerkinModule);
  // we need the solver type for the stepper's call operator
  galerkin_binder::bindImplicitSteppers<pressio4py::UserDefinedNonLinSolverWrapper,
					newraph_solver_t>(galerkinModule);

  pressio4py::BindAdvanceFunctions<galerkin_explicit_steppers>::fixedStepSize(odeModule);
  pressio4py::BindAdvanceFunctions<galerkin_explicit_steppers>::template arbitraryStepSize<
    pressio4py::ode_dt_setter_wrapper_type>(odeModule);

  // // for implicit galerkin, we need the solver
  // pressio4py::solvers::BindCreateHelperForTuple<
  //   newraphbinder_t, galerkin_implicit_steppers>::bindCreate(solversModule);

  pressio4py::BindAdvanceFunctions<
    galerkin_implicit_steppers>::template fixedStepSize<newraph_solver_t>(odeModule);
  pressio4py::BindAdvanceFunctions<
    galerkin_implicit_steppers>::template arbitraryStepSize<
      pressio4py::ode_dt_setter_wrapper_type, newraph_solver_t>(odeModule);

  pressio4py::BindAdvanceFunctions<
    galerkin_implicit_steppers>::template fixedStepSizeUserSolver<
      pressio4py::UserDefinedNonLinSolverWrapper>(odeModule);
  pressio4py::BindAdvanceFunctions<
    galerkin_implicit_steppers>::template arbitraryStepSizeUserSolver<
      pressio4py::ode_dt_setter_wrapper_type, pressio4py::UserDefinedNonLinSolverWrapper>(odeModule);

  // *** steady lspg ***
  using steady_lspg_binder = pressio4py::SteadyLSPGBinder;
  using steady_lspg_steppers = typename steady_lspg_binder::system_types;
  steady_lspg_binder::bindProblems(steadyLspgModule);

  // *** unsteady lspg ***
  using unsteady_lspg_binder = pressio4py::UnsteadyLSPGBinder;
  using unsteady_lspg_steppers = typename unsteady_lspg_binder::stepper_types;
  unsteady_lspg_binder::bindProblems(unsteadyLspgModule);
  unsteady_lspg_binder::bindSteppers<pressio4py::UserDefinedNonLinSolverWrapper,
  				     newraph_solver_t>(unsteadyLspgModule);

  pressio4py::BindAdvanceFunctions<
    unsteady_lspg_steppers>::template fixedStepSizeUserSolver<
      pressio4py::UserDefinedNonLinSolverWrapper>(odeModule);
  pressio4py::BindAdvanceFunctions<
    unsteady_lspg_steppers>::template arbitraryStepSizeUserSolver<
      pressio4py::ode_dt_setter_wrapper_type, pressio4py::UserDefinedNonLinSolverWrapper>(odeModule);

  pressio4py::BindAdvanceFunctions<
    unsteady_lspg_steppers>::template fixedStepSize<gn_neq_solver_t>(odeModule);
  pressio4py::BindAdvanceFunctions<
    unsteady_lspg_steppers>::template arbitraryStepSize<
      pressio4py::ode_dt_setter_wrapper_type, gn_neq_solver_t>(odeModule);
}

#endif
