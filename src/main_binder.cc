/*
//@HEADER
// ************************************************************************
//
// pressio4py.hpp
//                         Pressio
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

#ifndef PRESSIO4PY_PYBINDINGS_MAIN_BINDER_HPP_
#define PRESSIO4PY_PYBINDINGS_MAIN_BINDER_HPP_

#include <memory>
#include <stdexcept>
#include <utility>

#include "types.hpp"
#include "pressio/ode.hpp"
#include "pressio/rom.hpp"
#include "pressio/solvers_nonlinear.hpp"

#include "./wrappers/ode_system_wrapper.hpp"
#include "./logger.hpp"
#include "./rom/decoder.hpp"
#include "./rom/fomreconstructor.hpp"
#include "./nonlinear_solvers.hpp"

namespace pressio4py{

inline int ode_scheme_order(pressio::ode::StepScheme scheme)
{
  using pressio::ode::StepScheme;
  switch (scheme){
    case StepScheme::ForwardEuler: return 1;
    case StepScheme::RungeKutta4: return 4;
    case StepScheme::AdamsBashforth2: return 2;
    case StepScheme::SSPRungeKutta3: return 3;
    case StepScheme::BDF1: return 1;
    case StepScheme::BDF2: return 2;
    case StepScheme::CrankNicolson: return 2;
    default: throw std::runtime_error("unsupported ODE StepScheme");
  }
}

template<class SystemWrapper>
class ExplicitStepperAdapter
{
public:
  using independent_variable_type = scalar_t;
  using state_type = py_f_arr;

private:
  using native_stepper_type = decltype(
    pressio::ode::create_explicit_stepper(
      std::declval<pressio::ode::StepScheme>(),
      std::declval<SystemWrapper &>()));

  pressio::ode::StepScheme scheme_;
  std::unique_ptr<SystemWrapper> system_;
  native_stepper_type stepper_;

public:
  ExplicitStepperAdapter(pressio::ode::StepScheme scheme, pybind11::object pySystem)
    : scheme_(scheme),
      system_(std::make_unique<SystemWrapper>(pySystem)),
      stepper_(pressio::ode::create_explicit_stepper(scheme_, *system_))
  {
    (void) ode_scheme_order(scheme_);
  }

  ExplicitStepperAdapter() = delete;
  ExplicitStepperAdapter(const ExplicitStepperAdapter &) = delete;
  ExplicitStepperAdapter & operator=(const ExplicitStepperAdapter &) = delete;

  int order() const { return ode_scheme_order(scheme_); }

  void operator()(state_type & state,
                  pressio::ode::StepStartAt<scalar_t> time,
                  pressio::ode::StepCount step,
                  pressio::ode::StepSize<scalar_t> dt)
  {
    stepper_(state, time, step, dt);
  }

  void step(state_type & state, scalar_t time, scalar_t dt, int32_t step)
  {
    (*this)(state,
            pressio::ode::StepStartAt<scalar_t>(time),
            pressio::ode::StepCount(step),
            pressio::ode::StepSize<scalar_t>(dt));
  }
};


template<class SystemWrapper>
class ImplicitStepperAdapter
{
public:
  using independent_variable_type = scalar_t;
  using state_type = py_f_arr;
  using residual_type = py_f_arr;
  using jacobian_type = py_f_arr;

private:
  using native_stepper_type = decltype(
    pressio::ode::create_implicit_stepper(
      std::declval<pressio::ode::StepScheme>(),
      std::declval<SystemWrapper &>()));

  pressio::ode::StepScheme scheme_;
  std::unique_ptr<SystemWrapper> system_;
  native_stepper_type stepper_;

public:
  ImplicitStepperAdapter(pressio::ode::StepScheme scheme, pybind11::object pySystem)
    : scheme_(scheme),
      system_(std::make_unique<SystemWrapper>(pySystem)),
      stepper_(pressio::ode::create_implicit_stepper(scheme_, *system_))
  {
    (void) ode_scheme_order(scheme_);
  }

  ImplicitStepperAdapter() = delete;
  ImplicitStepperAdapter(const ImplicitStepperAdapter &) = delete;
  ImplicitStepperAdapter & operator=(const ImplicitStepperAdapter &) = delete;

  int order() const { return ode_scheme_order(scheme_); }

  state_type createState() const { return stepper_.createState(); }
  residual_type createResidual() const { return stepper_.createResidual(); }
  jacobian_type createJacobian() const { return stepper_.createJacobian(); }

  void residualAndJacobian(const state_type & state,
                           residual_type & residual,
                           std::optional<jacobian_type*> jacobian) const
  {
    stepper_.residualAndJacobian(state, residual, jacobian);
  }

  void residual(const state_type & state, residual_type & residual) const
  {
    stepper_.residualAndJacobian(state, residual, std::nullopt);
  }

  void jacobian(const state_type & state, jacobian_type & jacobian) const
  {
    auto residual = stepper_.createResidual();
    stepper_.residualAndJacobian(state, residual, std::optional<jacobian_type*>{&jacobian});
  }

  template<class SolverType>
  void operator()(state_type & state,
                  pressio::ode::StepStartAt<scalar_t> time,
                  pressio::ode::StepCount step,
                  pressio::ode::StepSize<scalar_t> dt,
                  SolverType & solver)
  {
    stepper_(state, time, step, dt, solver);
  }
};


class PythonNonlinearSolverBridge
{
  pybind11::object pySolver_;
  pybind11::object pySystem_;

public:
  PythonNonlinearSolverBridge(pybind11::object pySolver, pybind11::object pySystem)
    : pySolver_(std::move(pySolver)), pySystem_(std::move(pySystem)){}

  template<class NativeSystemType, class StateType>
  void solve(NativeSystemType &, StateType & state)
  {
    pySolver_.attr("solve")(pySystem_, state);
  }
};


template<class StepperType>
pybind11::object as_python_stepper(StepperType & stepper)
{
  return pybind11::cast(&stepper, pybind11::return_value_policy::reference);
}

} // end namespace pressio4py


PYBIND11_MODULE(MODNAME, topLevelModule)
{
  pybind11::module solversModule      = topLevelModule.def_submodule("solvers");
  pybind11::module odeModule          = topLevelModule.def_submodule("ode");
  pybind11::module romModule          = topLevelModule.def_submodule("rom");

  // Keep the historical module hierarchy importable during the staged 0.17
  // port.  Galerkin and LSPG bindings are intentionally compiled in follow-on
  // issues rather than blocking the core solver/ODE migration in issue #31.
  romModule.def_submodule("galerkin");
  pybind11::module lspgModule = romModule.def_submodule("lspg");
  lspgModule.def_submodule("steady");
  lspgModule.def_submodule("unsteady");

  pressio4py::bindLogger(topLevelModule);

  // =========================
  // NONLINEAR SOLVERS
  // =========================
  pressio4py::solvers::bindUpdatingEnums(solversModule);
  pressio4py::solvers::bindStoppingEnums(solversModule);

  using rj_system_type = pressio4py::ResJacInterface<
    pressio4py::scalar_t,
    pressio4py::py_f_arr,
    pressio4py::py_f_arr,
    pressio4py::py_f_arr>;

  using newraphbinder_t = pressio4py::solvers::NewtonRaphsonBinder<
    pressio4py::linear_solver_wrapper_t, rj_system_type>;
  newraphbinder_t::bindClassAndMethods(solversModule);

  using gnbinder_t = pressio4py::solvers::GNNormalEqResJacApiBinder<
    pressio4py::linear_solver_wrapper_t, rj_system_type>;
  gnbinder_t::bindClassAndMethods(solversModule);

  using wgnbinder_t = pressio4py::solvers::WeighGNNormalEqResJacApiBinder<
    pressio4py::linear_solver_wrapper_t,
    rj_system_type,
    pressio4py::nonlin_ls_weigh_wrapper_t>;
  wgnbinder_t::bindClassAndMethods(solversModule);

  using gnqrbinder_t = pressio4py::solvers::GNQRResJacApiBinder<
    pressio4py::qr_solver_wrapper_t, rj_system_type>;
  gnqrbinder_t::bindClassAndMethods(solversModule);

  using lmbinder_t = pressio4py::solvers::LMNormalEqResJacApiBinder<
    pressio4py::linear_solver_wrapper_t, rj_system_type>;
  lmbinder_t::bindClassAndMethods(solversModule);

  using wlmbinder_t = pressio4py::solvers::WeighLMNormalEqResJacApiBinder<
    pressio4py::linear_solver_wrapper_t,
    rj_system_type,
    pressio4py::nonlin_ls_weigh_wrapper_t>;
  wlmbinder_t::bindClassAndMethods(solversModule);

  // =========================
  // ODE
  // =========================
  pybind11::enum_<pressio::ode::StepScheme>(odeModule, "stepscheme")
    .value("ForwardEuler", pressio::ode::StepScheme::ForwardEuler)
    .value("RungeKutta4", pressio::ode::StepScheme::RungeKutta4)
    .value("AdamsBashforth2", pressio::ode::StepScheme::AdamsBashforth2)
    .value("SSPRungeKutta3", pressio::ode::StepScheme::SSPRungeKutta3)
    .value("BDF1", pressio::ode::StepScheme::BDF1)
    .value("BDF2", pressio::ode::StepScheme::BDF2)
    .value("CrankNicolson", pressio::ode::StepScheme::CrankNicolson)
    .value("ImplicitArbitrary", pressio::ode::StepScheme::ImplicitArbitrary)
    .export_values();

  using explicit_system_t = pressio4py::OdeSystemExplicitWrapper<
    pressio4py::scalar_t, pressio4py::py_f_arr, pressio4py::py_f_arr>;
  using explicit_stepper_t = pressio4py::ExplicitStepperAdapter<explicit_system_t>;

  pybind11::class_<explicit_stepper_t> explicitStepper(odeModule, "ExplicitStepper");
  explicitStepper.def("order", &explicit_stepper_t::order)
    .def("__call__", &explicit_stepper_t::step, pybind11::is_operator());

  odeModule.def("create_explicit_stepper",
    [](pressio::ode::StepScheme scheme,
       const pressio4py::py_f_arr &,
       pybind11::object system){
      return std::make_unique<explicit_stepper_t>(scheme, std::move(system));
    });

  odeModule.def("advance_n_steps",
    [](explicit_stepper_t & stepper,
       pressio4py::py_f_arr & state,
       pressio4py::scalar_t startTime,
       pressio4py::scalar_t dt,
       int32_t numSteps){
      auto policy = pressio::ode::steps_fixed_dt(
        startTime, pressio::ode::StepCount(numSteps), dt);
      pressio::ode::advance(stepper, state, policy);
    });

  odeModule.def("advance_n_steps",
    [](explicit_stepper_t & stepper,
       pressio4py::py_f_arr & state,
       pressio4py::scalar_t startTime,
       pybind11::object dtSetter,
       int32_t numSteps){
      pressio4py::ode_dt_setter_wrapper_type setter(std::move(dtSetter));
      auto policy = pressio::ode::steps(
        startTime, pressio::ode::StepCount(numSteps), setter);
      pressio::ode::advance(stepper, state, policy);
    });

  odeModule.def("advance_n_steps_and_observe",
    [](explicit_stepper_t & stepper,
       pressio4py::py_f_arr & state,
       pressio4py::scalar_t startTime,
       pressio4py::scalar_t dt,
       int32_t numSteps,
       pybind11::object observer){
      pressio4py::ode_observer_wrapper_type obs(std::move(observer));
      auto policy = pressio::ode::steps_fixed_dt(
        startTime, pressio::ode::StepCount(numSteps), dt);
      pressio::ode::advance(stepper, state, policy, obs);
    });

  odeModule.def("advance_n_steps_and_observe",
    [](explicit_stepper_t & stepper,
       pressio4py::py_f_arr & state,
       pressio4py::scalar_t startTime,
       pybind11::object dtSetter,
       int32_t numSteps,
       pybind11::object observer){
      pressio4py::ode_dt_setter_wrapper_type setter(std::move(dtSetter));
      pressio4py::ode_observer_wrapper_type obs(std::move(observer));
      auto policy = pressio::ode::steps(
        startTime, pressio::ode::StepCount(numSteps), setter);
      pressio::ode::advance(stepper, state, policy, obs);
    });

  using implicit_system_t = pressio4py::OdeSystemImplicitContTimeWrapper<
    pressio4py::scalar_t,
    pressio4py::py_f_arr,
    pressio4py::py_f_arr,
    pressio4py::py_f_arr>;
  using implicit_stepper_t = pressio4py::ImplicitStepperAdapter<implicit_system_t>;

  pybind11::class_<implicit_stepper_t> implicitStepper(odeModule, "ImplicitStepper");
  implicitStepper.def("order", &implicit_stepper_t::order)
    .def("createResidual", &implicit_stepper_t::createResidual)
    .def("createJacobian", &implicit_stepper_t::createJacobian)
    .def("residual", &implicit_stepper_t::residual)
    .def("jacobian", &implicit_stepper_t::jacobian)
    .def("__call__",
      [](implicit_stepper_t & stepper,
         pressio4py::py_f_arr & state,
         pressio4py::scalar_t time,
         pressio4py::scalar_t dt,
         int32_t step,
         pybind11::object solver){
        pressio4py::PythonNonlinearSolverBridge bridge(
          std::move(solver), pressio4py::as_python_stepper(stepper));
        stepper(state,
                pressio::ode::StepStartAt<pressio4py::scalar_t>(time),
                pressio::ode::StepCount(step),
                pressio::ode::StepSize<pressio4py::scalar_t>(dt),
                bridge);
      }, pybind11::is_operator());

  odeModule.def("create_implicit_stepper",
    [](pressio::ode::StepScheme scheme,
       const pressio4py::py_f_arr &,
       pybind11::object system){
      return std::make_unique<implicit_stepper_t>(scheme, std::move(system));
    });

  odeModule.def("advance_n_steps",
    [](implicit_stepper_t & stepper,
       pressio4py::py_f_arr & state,
       pressio4py::scalar_t startTime,
       pressio4py::scalar_t dt,
       int32_t numSteps,
       pybind11::object solver){
      pressio4py::PythonNonlinearSolverBridge bridge(
        std::move(solver), pressio4py::as_python_stepper(stepper));
      auto policy = pressio::ode::steps_fixed_dt(
        startTime, pressio::ode::StepCount(numSteps), dt);
      pressio::ode::advance(stepper, state, policy, bridge);
    });

  odeModule.def("advance_n_steps",
    [](implicit_stepper_t & stepper,
       pressio4py::py_f_arr & state,
       pressio4py::scalar_t startTime,
       pybind11::object dtSetter,
       int32_t numSteps,
       pybind11::object solver){
      pressio4py::ode_dt_setter_wrapper_type setter(std::move(dtSetter));
      pressio4py::PythonNonlinearSolverBridge bridge(
        std::move(solver), pressio4py::as_python_stepper(stepper));
      auto policy = pressio::ode::steps(
        startTime, pressio::ode::StepCount(numSteps), setter);
      pressio::ode::advance(stepper, state, policy, bridge);
    });

  odeModule.def("advance_n_steps_and_observe",
    [](implicit_stepper_t & stepper,
       pressio4py::py_f_arr & state,
       pressio4py::scalar_t startTime,
       pressio4py::scalar_t dt,
       int32_t numSteps,
       pybind11::object observer,
       pybind11::object solver){
      pressio4py::ode_observer_wrapper_type obs(std::move(observer));
      pressio4py::PythonNonlinearSolverBridge bridge(
        std::move(solver), pressio4py::as_python_stepper(stepper));
      auto policy = pressio::ode::steps_fixed_dt(
        startTime, pressio::ode::StepCount(numSteps), dt);
      pressio::ode::advance(stepper, state, policy, bridge, obs);
    });

  odeModule.def("advance_n_steps_and_observe",
    [](implicit_stepper_t & stepper,
       pressio4py::py_f_arr & state,
       pressio4py::scalar_t startTime,
       pybind11::object dtSetter,
       int32_t numSteps,
       pybind11::object observer,
       pybind11::object solver){
      pressio4py::ode_dt_setter_wrapper_type setter(std::move(dtSetter));
      pressio4py::ode_observer_wrapper_type obs(std::move(observer));
      pressio4py::PythonNonlinearSolverBridge bridge(
        std::move(solver), pressio4py::as_python_stepper(stepper));
      auto policy = pressio::ode::steps(
        startTime, pressio::ode::StepCount(numSteps), setter);
      pressio::ode::advance(stepper, state, policy, bridge, obs);
    });

  // =========================
  // ROM CORE
  // =========================
  pressio4py::rom::bindDecoder(romModule);
  pressio4py::rom::bindFomReconstructor(romModule);
}

#endif
