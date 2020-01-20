/*
//@HEADER
// ************************************************************************
//
// lspg.hpp
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

#ifndef PRESSIO4PPY_PYBINDINGS_LSPG_HPP_
#define PRESSIO4PY_PYBINDINGS_LSPG_HPP_

#include "UTILS_ALL"
#include "CONTAINERS_ALL"
#include "SOLVERS_NONLINEAR"
#include "ODE_ALL"
#include "ROM_LSPG_UNSTEADY"

#include "types.hpp"

PYBIND11_MODULE(pressio4pyLspg, m) {

  using mytypes		= DefaultLSPGTypes;
  using scalar_t	= typename mytypes::scalar_t;
  using fom_t		= typename mytypes::fom_t;
  using ops_t		= typename mytypes::ops_t;

  using rom_state_t	= typename mytypes::rom_state_t;
  using fom_state_t	= typename mytypes::fom_state_t;
  using decoder_jac_t	= typename mytypes::decoder_jac_t;
  using hessian_t	= typename mytypes::hessian_t;

  // --------------------------------------------------------------------
  // decoder
  // --------------------------------------------------------------------
  using decoder_t	= ::pressio::rom::PyLinearDecoder<decoder_jac_t, ops_t>;

  // base decoder class
  using decoder_base_t = ::pressio::rom::DecoderBase<decoder_t, decoder_jac_t>;
  pybind11::class_<decoder_base_t>(m, "DecoderBase")
    .def("applyMapping", &decoder_base_t::template applyMapping<rom_state_t, fom_state_t>);

  pybind11::class_<decoder_t, decoder_base_t>(m, "LinearDecoder")
    .def(pybind11::init< const decoder_jac_t &>());

  // --------------------------------------------------------------------
  // ---- lspg problem ----
  constexpr auto ode_case  = ::pressio::ode::ImplicitEnum::Euler;
  using lspg_problem_type  = pressio::rom::lspg::unsteady::Problem<
    pressio::rom::lspg::unsteady::Default, ode_case, fom_t, rom_state_t, decoder_t, ops_t>;

  // extract types from the lspg problem type
  using lspg_stepper_t	= typename lspg_problem_type::lspg_stepper_t;
  using res_pol_t	= typename lspg_problem_type::lspg_residual_policy_t;
  using jac_pol_t	= typename lspg_problem_type::lspg_jacobian_policy_t;

  // concrete LSPG stepper binding
  pybind11::class_<lspg_stepper_t>(m, "StepperEuler")
    .def(pybind11::init<const rom_state_t &, const fom_t &, const res_pol_t &, const jac_pol_t &>());

  // concrete LSPG problem
  pybind11::class_<lspg_problem_type>(m, "ProblemEuler")
    .def(pybind11::init<const fom_t &, const fom_state_t &, const decoder_t &, rom_state_t &, scalar_t>())
    .def("getStepper", &lspg_problem_type::getStepperRef);

  // --------------------------------------------------------------------
  // ---- linear solver type ----
  // use pybind::object because the solver is actually defined by the user
  using linear_solver_t = pybind11::object;

  // ---- non-linear solver type ---
  using nonlin_solver_t = ::pressio::solvers::iterative::PyGaussNewton
    <lspg_stepper_t, rom_state_t, rom_state_t, decoder_jac_t, hessian_t, linear_solver_t, scalar_t, ops_t>;

  // base types
  using nls_base_t = ::pressio::solvers::NonLinearSolverBase<nonlin_solver_t>;
  using iter_base_t = ::pressio::solvers::IterativeBase<nonlin_solver_t, scalar_t>;

  pybind11::class_<nls_base_t>(m, "NonLinSolverBase")
    .def("solve", &nls_base_t::template solve<lspg_stepper_t, rom_state_t>);

  pybind11::class_<iter_base_t>(m, "IterBase")
    .def("getMaxIterations", &iter_base_t::getMaxIterations)
    .def("setMaxIterations", &iter_base_t::setMaxIterations)
    .def("getTolerance", &iter_base_t::getTolerance)
    .def("setTolerance", &iter_base_t::setTolerance);

  pybind11::class_<nonlin_solver_t, iter_base_t, nls_base_t>(m, "GaussNewton")
    .def(pybind11::init<const lspg_stepper_t &, const rom_state_t &, linear_solver_t &>());

  // integrator
  m.def("integrateNSteps",
  	&::pressio::ode::integrateNSteps<lspg_stepper_t, rom_state_t, scalar_t, nonlin_solver_t>,
  	"Integrate N Steps");

}
#endif
