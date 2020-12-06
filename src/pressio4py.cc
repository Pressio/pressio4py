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
#include <pybind11/stl.h>

// pressio include
#include "pressio_rom.hpp"

// local includes
#include "types.hpp"
#include "./rom/wrappers/fom_steady_wrapper.hpp"
#include "./rom/wrappers/fom_continuous_time_wrapper.hpp"
#include "./rom/wrappers/lin_solver_wrapper.hpp"
#include "./rom/wrappers/qr_solver_wrapper.hpp"
#include "./rom/wrappers/ode_collector_wrapper.hpp"
#include "./rom/wrappers/nonlin_ls_weighting_wrapper.hpp"

#include "./rom/decoder.hpp"
#include "./rom/fomreconstructor.hpp"
#include "./rom/galerkin.hpp"
#include "./rom/lspg.hpp"
#include "./rom/nonlinear_solvers.hpp"

#include "./solve_problem_api_bind.hpp"

PYBIND11_MODULE(pressio4py, mParent)
{
  // create the rom submodule
  pybind11::module romModule = mParent.def_submodule("rom");

  // some aliases neeeded below
  using scalar_t	   = typename pressio4py::ROMTypes::scalar_t;
  using rom_native_state_t = typename pressio4py::ROMTypes::rom_native_state_t;
  using rom_state_t	   = typename pressio4py::ROMTypes::rom_state_t;

  //---------------------
  // decoder bindings
  //---------------------
  pressio4py::createDecoderBindings<pressio4py::ROMTypes>(romModule);

  //---------------------
  // fom reconstructor bindings
  //---------------------
  pressio4py::createFomReconstructorBindings<pressio4py::ROMTypes>(romModule);

  //---------------------
  //  galerkin rom
  //---------------------
  pybind11::module galerkinModule= romModule.def_submodule("galerkin");
  using galerkin_binder_t = pressio4py::rom::GalerkinBinder<pressio4py::ROMTypes>;
  galerkin_binder_t::bind(galerkinModule);

  //---------------------
  //  lspg rom
  // de: stands for default
  // hr: stands for hyper-reduced
  //---------------------
  pybind11::module lspgModule	= romModule.def_submodule("lspg");
  using lspg_binder_t		= pressio4py::rom::LSPGBinder<pressio4py::ROMTypes>;
  lspg_binder_t::bind(lspgModule);
  // need to extract problem types to bind all the solver methods below
  using lspgproblems		= typename lspg_binder_t::problem_types;
  using steady_lspgproblems	= typename lspg_binder_t::steady_problem_types;
  using unsteady_lspgproblems	= typename lspg_binder_t::unsteady_problem_types;

  //---------------------
  //	solvers
  //---------------------
  pybind11::module mSolver = mParent.def_submodule("solvers");

  // updating and stopping enums are common to all, so just do it once
  pressio4py::solvers::bindUpdatingEnums(mSolver);
  pressio4py::solvers::bindStoppingEnums(mSolver);

  // auxiliary types needed for various solvers, e.g. hessian, jac_t, etc
  using hessian_t	= typename pressio4py::ROMTypes::lsq_hessian_t;
  using jac_t           = typename pressio4py::ROMTypes::decoder_jac_t;
  using linear_solver_t = pressio4py::LinSolverWrapper<hessian_t>;
  using qr_solver_t     = pressio4py::QrSolverWrapper<jac_t>;
  using nlls_weigher_t  = pressio4py::NonLinLSWeightingWrapper;

  // GN with normal equations
  using gnbinder_t =
    typename pressio4py::solvers::instantiate_from_tuple_pack<
      pressio4py::solvers::LeastSquaresNormalEqBinder,
    true, linear_solver_t, lspgproblems>::type;
  using gn_solver_t = typename gnbinder_t::nonlinear_solver_t;
  gnbinder_t::bind(mSolver, "GaussNewton");

  // GN with QR
  using gnbinder_qr_t =
    typename pressio4py::solvers::instantiate_from_tuple_pack<
      pressio4py::solvers::LeastSquaresQRBinder,
    true, qr_solver_t, lspgproblems>::type;
  using gn_qr_solver_t = typename gnbinder_qr_t::nonlinear_solver_t;
  gnbinder_qr_t::bind(mSolver, "GaussNewtonQR");

  // weighted GN with normal equations
  using wgnbinder_t =
    typename pressio4py::solvers::instantiate_from_tuple_pack<
      pressio4py::solvers::WeightedLeastSquaresNormalEqBinder,
    true, linear_solver_t,  nlls_weigher_t, lspgproblems>::type;
  using w_gn_solver_t = typename wgnbinder_t::nonlinear_solver_t;
  wgnbinder_t::bind(mSolver, "WeightedGaussNewton");

  // IRW GN with normal equations
  using irwgnbinder_t =
    typename pressio4py::solvers::instantiate_from_tuple_pack<
    pressio4py::solvers::IrwLeastSquaresNormalEqBinder,
    true, linear_solver_t,  lspgproblems>::type;
  using irwgn_solver_t = typename irwgnbinder_t::nonlinear_solver_t;
  irwgnbinder_t::bind(mSolver, "IrwGaussNewton");

  // LM with normal equations
  using lmbinder_t =
    typename pressio4py::solvers::instantiate_from_tuple_pack<
    pressio4py::solvers::LeastSquaresNormalEqBinder,
    false, linear_solver_t, lspgproblems>::type;
  using lm_solver_t = typename lmbinder_t::nonlinear_solver_t;
  lmbinder_t::bind(mSolver, "LevenbergMarquardt");

  // create tuple with all sover types
  using solvers = std::tuple<gn_solver_t, irwgn_solver_t,
			     w_gn_solver_t, gn_qr_solver_t, lm_solver_t>;

  //-------------------------------------
  // exposed api to solve rom problems
  //-------------------------------------
  // collector used to monitor the rom state
  using collector_t = pressio4py::OdeCollectorWrapper<rom_state_t>;

  // functions for galerkin: here we do this manually but
  // should be done similarly to lspg below to make this more automated
  galerkinModule.def("advanceNSteps", // for Galerkin Euler
		     &pressio::rom::galerkin::solveNSteps<
		     typename galerkin_binder_t::problem_euler_t,
		     rom_native_state_t, scalar_t, collector_t>);
  galerkinModule.def("advanceNSteps", // for Galerkin RK4
		     &pressio::rom::galerkin::solveNSteps<
		     typename galerkin_binder_t::problem_rk4_t,
		     rom_native_state_t, scalar_t, collector_t>);

  // for lspg, we have MANY solver choices and MANY problem types,
  // so writing explicitly the binding for solving all these is verbose.
  // So we use some metaprogramming to facilitate binding each problem to each problem.
  pressio4py::bindLspgProbsWithMultipleSolvers
    <steady_lspgproblems, solvers>::template bind<rom_native_state_t>(lspgModule);
  // note that for unsteady we need to passs the collector object
  pressio4py::bindLspgProbsWithMultipleSolvers
    <unsteady_lspgproblems, solvers>::template bind<rom_native_state_t, collector_t>(lspgModule);


  // TODO: not working because of conflicting std::out and sys.out
  // // logger
  // pybind11::module loggerModule = mParent.def_submodule("logger");
  // pybind11::enum_<pressio::logto>(loggerModule, "logto")
  //   .value("terminal",	      pressio::logto::terminal)
  //   .value("fileAndTerminal", pressio::logto::fileAndTerminal)
  //   .value("terminalAndFile", pressio::logto::terminalAndFile)
  //   .value("file", pressio::logto::file)
  //   .export_values();

  // pybind11::enum_<pressio::log::level>(loggerModule, "loglevel")
  //   .value("trace",	pressio::log::level::trace)
  //   .value("debug",	pressio::log::level::debug)
  //   .value("info",	pressio::log::level::info)
  //   .value("warn",	pressio::log::level::warn)
  //   .value("err",	pressio::log::level::err)
  //   .value("critical",	pressio::log::level::critical)
  //   .value("off",	pressio::log::level::off)
  //   .export_values();

  // loggerModule.def("initialize",
  //  		   &pressio::log::initialize);
  // loggerModule.def("setVerbosity",
  // 		   &pressio::log::setVerbosity<std::vector<::pressio::log::level>>);
}

#endif
