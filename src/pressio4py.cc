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

#ifndef PRESSIO4PY_PYBINDINGS_PRESSIO_FOUR_PY_HPP_
#define PRESSIO4PY_PYBINDINGS_PRESSIO_FOUR_PY_HPP_

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>

// pressio include
#include "pressio_rom.hpp"

// pressio4py includes
#include "types.hpp"

#include "./rom/wrappers/fom_steady_wrapper.hpp"
#include "./rom/wrappers/fom_continuous_time_wrapper.hpp"
#include "./rom/wrappers/lin_solver_wrapper.hpp"
#include "./rom/wrappers/ode_collector_wrapper.hpp"

#include "./rom/decoder.hpp"
#include "./rom/fomreconstructor.hpp"
#include "./rom/galerkin.hpp"
#include "./rom/lspg.hpp"
#include "./rom/nonlinear_solvers.hpp"

PYBIND11_MODULE(pressio4py, mParent)
{
  pybind11::module romModule = mParent.def_submodule("rom");
  using scalar_t	   = typename pressio4py::ROMTypes::scalar_t;
  using rom_native_state_t = typename pressio4py::ROMTypes::rom_native_state_t;
  using rom_state_t	   = typename pressio4py::ROMTypes::rom_state_t;

  // decoder bindings
  pressio4py::createDecoderBindings<pressio4py::ROMTypes>(romModule);

  // fom reconstructor bindings
  pressio4py::createFomReconstructorBindings<pressio4py::ROMTypes>(romModule);

  //---------------------
  //  rom methods
  //---------------------
  //*** galerkin ***
  pybind11::module galerkinModule= romModule.def_submodule("galerkin");
  using galerkin_binder_t = pressio4py::rom::GalerkinBinder<pressio4py::ROMTypes>;
  galerkin_binder_t::bind(galerkinModule);

  //*** lspg ***
  pybind11::module lspgModule	= romModule.def_submodule("lspg");
  using lspg_binder_t		= pressio4py::rom::LSPGBinder<pressio4py::ROMTypes>;
  //steady
  using lspg_steady_prob_t	= typename lspg_binder_t::lspg_steady_problem_t;
  //bdf1 default
  using lspg_de_prob_bdf1_t	= typename lspg_binder_t::de_lspg_problem_bdf1_t;
  //bdf1 hypred
  using lspg_hr_prob_bdf1_t	= typename lspg_binder_t::hr_lspg_problem_bdf1_t;
  //bdf2 default
  using lspg_de_prob_bdf2_t	= typename lspg_binder_t::de_lspg_problem_bdf2_t;
  //bdf2 hypred
  using lspg_hr_prob_bdf2_t	= typename lspg_binder_t::hr_lspg_problem_bdf2_t;
  // do bind
  lspg_binder_t::bind(lspgModule);

  //---------------------
  //	solvers
  //---------------------
  pybind11::module mSolver = mParent.def_submodule("solvers");

  pressio4py::solvers::bindUpdatingEnums(mSolver);
  pressio4py::solvers::bindStoppingEnums(mSolver);

  // for least-squares normal equations, we have a hessian and gradient
  using hessian_t = typename pressio4py::ROMTypes::lsq_hessian_t;
  using linear_solver_t = pressio4py::LinSolverWrapper<hessian_t>;

  // GN with normal equations
  using gnbinder_t = pressio4py::solvers::LeastSquaresNormalEqBinder<
    true,
    lspg_steady_prob_t,
    lspg_de_prob_bdf1_t, lspg_de_prob_bdf2_t,
    lspg_hr_prob_bdf1_t, lspg_hr_prob_bdf2_t,
    linear_solver_t, rom_native_state_t, rom_state_t>;
  using gn_solver_t = typename gnbinder_t::nonlinear_solver_t;
  gnbinder_t::bind(mSolver, "GaussNewton");

  // LM with normal equations
  using lmbinder_t = pressio4py::solvers::LeastSquaresNormalEqBinder<
    false,
    lspg_steady_prob_t,
    lspg_de_prob_bdf1_t, lspg_de_prob_bdf2_t,
    lspg_hr_prob_bdf1_t, lspg_hr_prob_bdf2_t,
    linear_solver_t, rom_native_state_t, rom_state_t>;
  using lm_solver_t = typename lmbinder_t::nonlinear_solver_t;
  lmbinder_t::bind(mSolver, "LevenbergMarquardt");

  //-------------------------------------
  // exposed api to solve rom problems
  //-------------------------------------
  // collector used to monitor the rom state
  using collector_t = pressio4py::OdeCollectorWrapper<rom_state_t>;

  // functions for galerkin
  galerkinModule.def("advanceNSteps", // for Galerkin Euler
		     &pressio::rom::galerkin::solveNSteps<
		     typename galerkin_binder_t::problem_euler_t,
		     rom_native_state_t, scalar_t, collector_t>);
  galerkinModule.def("advanceNSteps", // for Galerkin RK4
		     &pressio::rom::galerkin::solveNSteps<
		     typename galerkin_binder_t::problem_rk4_t,
		     rom_native_state_t, scalar_t, collector_t>);

  // for steady lspg
  lspgModule.def("solveSteady", // default with GN
		 &::pressio::rom::lspg::solveSteady<
		 lspg_steady_prob_t, rom_native_state_t, gn_solver_t>);
  lspgModule.def("solveSteady", // default with LM
   		 &::pressio::rom::lspg::solveSteady<
		 lspg_steady_prob_t, rom_native_state_t, lm_solver_t>);

  // unsteady lspg bdf1
  lspgModule.def("solveNSequentialMinimizations", // default with GN
		 &::pressio::rom::lspg::solveNSequentialMinimizations<
		 lspg_de_prob_bdf1_t, rom_native_state_t,
		 scalar_t, collector_t, gn_solver_t>);
  lspgModule.def("solveNSequentialMinimizations", // hyp-red with GN
		 &::pressio::rom::lspg::solveNSequentialMinimizations<
		 lspg_hr_prob_bdf1_t, rom_native_state_t,
		 scalar_t, collector_t, gn_solver_t>);
  lspgModule.def("solveNSequentialMinimizations", // default with LM
		 &::pressio::rom::lspg::solveNSequentialMinimizations<
		 lspg_de_prob_bdf1_t, rom_native_state_t,
		 scalar_t, collector_t, lm_solver_t>);
  lspgModule.def("solveNSequentialMinimizations", // hyp-red with LM
		 &::pressio::rom::lspg::solveNSequentialMinimizations<
		 lspg_hr_prob_bdf1_t, rom_native_state_t,
		 scalar_t, collector_t, lm_solver_t>);

  // unsteady lspg bdf2
  lspgModule.def("solveNSequentialMinimizations", // default with GN
		 &::pressio::rom::lspg::solveNSequentialMinimizations<
		 lspg_de_prob_bdf2_t, rom_native_state_t,
		 scalar_t, collector_t, gn_solver_t>);
  lspgModule.def("solveNSequentialMinimizations", // hyp-red with GN
		 &::pressio::rom::lspg::solveNSequentialMinimizations<
		 lspg_hr_prob_bdf2_t, rom_native_state_t,
		 scalar_t, collector_t, gn_solver_t>);
  lspgModule.def("solveNSequentialMinimizations", // default with LM
		 &::pressio::rom::lspg::solveNSequentialMinimizations<
		 lspg_de_prob_bdf2_t, rom_native_state_t,
		 scalar_t, collector_t, lm_solver_t>);
  lspgModule.def("solveNSequentialMinimizations", // hyp-red with LM
		 &::pressio::rom::lspg::solveNSequentialMinimizations<
		 lspg_hr_prob_bdf2_t, rom_native_state_t,
		 scalar_t, collector_t, lm_solver_t>);
}

#endif
