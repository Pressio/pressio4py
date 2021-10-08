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

#ifndef PRESSIO4PY_PYBINDINGS_ADVANCE_FUNCTIONS_HPP_
#define PRESSIO4PY_PYBINDINGS_ADVANCE_FUNCTIONS_HPP_

namespace pressio4py{

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

}//end namespace pressio4py
#endif
