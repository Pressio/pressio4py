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

} //end namespace pressio4py

#include "./advance_functions.hpp"
#include "./galerkin.hpp"
#include "./lspg_steady.hpp"
#include "./lspg_unsteady.hpp"

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
  using galerkin_explicit_problems = typename galerkin_binder::explicit_problem_types;
  using galerkin_implicit_problems = typename galerkin_binder::implicit_problem_types;
  galerkin_binder::bindExplicitProblems(galerkinModule);
  galerkin_binder::bindImplicitProblems<
    pressio4py::UserDefinedNonLinSolverWrapper, newraph_solver_t>(galerkinModule);

  pressio4py::BindAdvanceFunctions<galerkin_explicit_problems>::fixedStepSize(odeModule);
  pressio4py::BindAdvanceFunctions<galerkin_explicit_problems>::template arbitraryStepSize<
    pressio4py::ode_dt_setter_wrapper_type>(odeModule);

  pressio4py::BindAdvanceFunctions<
    galerkin_implicit_problems>::template fixedStepSize<newraph_solver_t>(odeModule);
  pressio4py::BindAdvanceFunctions<
    galerkin_implicit_problems>::template arbitraryStepSize<
      pressio4py::ode_dt_setter_wrapper_type, newraph_solver_t>(odeModule);

  pressio4py::BindAdvanceFunctions<
    galerkin_implicit_problems>::template fixedStepSizeUserSolver<
      pressio4py::UserDefinedNonLinSolverWrapper>(odeModule);
  pressio4py::BindAdvanceFunctions<
    galerkin_implicit_problems>::template arbitraryStepSizeUserSolver<
      pressio4py::ode_dt_setter_wrapper_type, pressio4py::UserDefinedNonLinSolverWrapper>(odeModule);

  // *** steady lspg ***
  using steady_lspg_binder = pressio4py::SteadyLSPGBinder;
  using steady_lspg_problems = typename steady_lspg_binder::system_types;
  steady_lspg_binder::bindProblems(steadyLspgModule);

  // *** unsteady lspg ***
  using unsteady_lspg_binder = pressio4py::UnsteadyLSPGBinder;
  using unsteady_lspg_problems = typename unsteady_lspg_binder::problem_types;
  unsteady_lspg_binder::bindProblems<pressio4py::UserDefinedNonLinSolverWrapper,
				     newraph_solver_t>(unsteadyLspgModule);

  pressio4py::BindAdvanceFunctions<
    unsteady_lspg_problems>::template fixedStepSizeUserSolver<
      pressio4py::UserDefinedNonLinSolverWrapper>(odeModule);
  pressio4py::BindAdvanceFunctions<
    unsteady_lspg_problems>::template arbitraryStepSizeUserSolver<
      pressio4py::ode_dt_setter_wrapper_type, pressio4py::UserDefinedNonLinSolverWrapper>(odeModule);

  pressio4py::BindAdvanceFunctions<
    unsteady_lspg_problems>::template fixedStepSize<gn_neq_solver_t>(odeModule);
  pressio4py::BindAdvanceFunctions<
    unsteady_lspg_problems>::template arbitraryStepSize<
      pressio4py::ode_dt_setter_wrapper_type, gn_neq_solver_t>(odeModule);
}

#endif
