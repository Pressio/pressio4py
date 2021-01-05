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
#include <pybind11/iostream.h>
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
#include "./rom/wrappers/masker_wrapper.hpp"

#include "./rom/decoder.hpp"
#include "./rom/fomreconstructor.hpp"
#include "./rom/galerkin.hpp"
#include "./rom/lspg.hpp"
#include "./rom/nonlinear_solvers.hpp"

#include "./lspg_solve_problem_api_bind.hpp"
#include "./galerkin_solve_problem_api_bind.hpp"

#define PPSTRINGIFY(x) #x
#define PPMACRO_STRINGIFY(x) PPSTRINGIFY(x)

PYBIND11_MODULE(pressio4py, mParent)
{
  mParent.attr("__version__") = PPMACRO_STRINGIFY(VERSION_IN);

  //*****************
  // bind logging
  //*****************
  pybind11::module loggerModule = mParent.def_submodule("logger");
  pybind11::enum_<pressio::logto>(loggerModule, "logto")
    .value("terminal", pressio::logto::terminal)
    .export_values();

  pybind11::enum_<pressio::log::level>(loggerModule, "loglevel")
    .value("trace",	pressio::log::level::trace)
    .value("debug",	pressio::log::level::debug)
    .value("info",	pressio::log::level::info)
    .value("warn",	pressio::log::level::warn)
    .value("err",	pressio::log::level::err)
    .value("critical",	pressio::log::level::critical)
    .value("off",	pressio::log::level::off)
    .export_values();

  // make sure to redirect stdout/stderr streams to python stdout
  pybind11::add_ostream_redirect(mParent, "ostream_redirect");

  // bind the initialization and setVerbosity functions
  loggerModule.def("initialize",
   		   &pressio::log::initialize);
  loggerModule.def("setVerbosity",
		   &pressio::log::setVerbosity<std::vector<::pressio::log::level>>);

  //*****************
  // bind ROM
  //*****************
  pybind11::module romModule = mParent.def_submodule("rom");

  pybind11::module r2m = romModule.def_submodule("rank2state");
  pybind11::module r3m = romModule.def_submodule("rank3state");
  //---------------------
  // decoder bindings
  //---------------------
  pressio4py::createDecoderBindings<pressio4py::DecoderTypes<1,2>>(romModule, "Decoder");
  pressio4py::createDecoderBindings<pressio4py::DecoderTypes<2,2>>(r2m,	      "Decoder");
  pressio4py::createDecoderBindings<pressio4py::DecoderTypes<2,3>>(r2m,	      "MultiFieldDecoder");
  pressio4py::createDecoderBindings<pressio4py::DecoderTypes<3,3>>(r3m,	      "MultiFieldDecoder");

  //---------------------------
  // fom reconstructor bindings
  //---------------------------
  pressio4py::createFomReconstructorBindings<pressio4py::DecoderTypes<1,2>>(romModule, "FomReconstructor");
  pressio4py::createFomReconstructorBindings<pressio4py::DecoderTypes<2,2>>(r2m,       "FomReconstructor");
  pressio4py::createFomReconstructorBindings<pressio4py::DecoderTypes<2,3>>(r2m,       "MultiFieldFomReconstructor");
  pressio4py::createFomReconstructorBindings<pressio4py::DecoderTypes<3,3>>(r3m,       "MultiFieldFomReconstructor");

  //--------------------------------
  // galerkin rom
  //--------------------------------
  pybind11::module galerkinModule = romModule.def_submodule("galerkin");
  pybind11::module galerkinMultiFieldModule = galerkinModule.def_submodule("multifield");

  // projector
  pressio4py::rom::bindProjector<pressio4py::GalerkinTypes<1,2>>(galerkinModule,"ArbitraryProjector");
  pressio4py::rom::bindProjector<pressio4py::GalerkinTypes<2,3>>(galerkinMultiFieldModule, "ArbitraryProjector");

  // explicit: rank-1 state and rank-2 decoder
  using galerkin_12_explicit_binder_t = pressio4py::rom::GalerkinBinderExplicit<pressio4py::GalerkinTypes<1,2>>;
  galerkin_12_explicit_binder_t::bind(galerkinModule);
  using explicit_12_galerkinproblems = typename galerkin_12_explicit_binder_t::problem_types;

  // explicit: rank-2 state and rank-3 decoder (i.e. multifield)
  using galerkin_23_explicit_binder_t = pressio4py::rom::GalerkinBinderExplicit<pressio4py::GalerkinTypes<2,3>>;
  galerkin_23_explicit_binder_t::bind(galerkinMultiFieldModule);
  using explicit_23_galerkinproblems = typename galerkin_23_explicit_binder_t::problem_types;

  // implicit (only supports rank-1 state and rank-2 decoder)
  using galerkin_implicit_binder_t  = pressio4py::rom::GalerkinBinderImplicit<pressio4py::GalerkinTypes<1,2>>;
  galerkin_implicit_binder_t::bind(galerkinModule);
  using implicit_galerkinproblems = typename galerkin_implicit_binder_t::problem_types;

  //---------------------
  //  lspg rom
  //---------------------
  pybind11::module lspgModule	= romModule.def_submodule("lspg");
  using lspg_binder_t		= pressio4py::rom::LSPGBinder<pressio4py::LspgTypes>;
  lspg_binder_t::bind(lspgModule);
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

  // newton-raphson
  using head_problem_t = typename std::tuple_element<0, implicit_galerkinproblems>::type;
  // we should make sure that all problems have same jacobian_t
  using galerkin_jacobian_t = typename head_problem_t::galerkin_jacobian_t;
  using linear_solver_nr_t = pressio4py::LinSolverWrapper<galerkin_jacobian_t>;
  using newraphbinder_t =
    typename pressio4py::solvers::instantiate_from_tuple_pack<
      pressio4py::solvers::NewtonRaphsonBinder,
    true, linear_solver_nr_t, implicit_galerkinproblems>::type;
  using newraph_solver_t = typename newraphbinder_t::nonlinear_solver_t;
  newraphbinder_t::bind(mSolver, "NewtonRaphson");

  // auxiliary types needed for nonlinear least-squares solvers
  using hessian_t	= typename pressio4py::LspgTypes::lsq_hessian_t;
  using linear_solver_t = pressio4py::LinSolverWrapper<hessian_t>;
  using jac_t           = typename pressio4py::LspgTypes::decoder_jac_t;
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
  using least_squares_solvers = std::tuple<
    gn_solver_t, irwgn_solver_t, w_gn_solver_t,
    gn_qr_solver_t, lm_solver_t>;

  //---------------------------------------------------------------------
  // expose api to solve rom problems
  //
  // note that writing explicitly the binding code for solving all problems
  // with all possible solver combinations is not practical.
  // So we use some metaprogramming to simplify the binding code generation.
  //---------------------------------------------------------------------
  using rom_rank1_state_t  = typename pressio4py::StateTypes<1>::rom_state_t;
  using rom_rank2_state_t  = typename pressio4py::StateTypes<2>::rom_state_t;
  using rom_rank3_state_t  = typename pressio4py::StateTypes<3>::rom_state_t;

  // collector used to monitor the rom state
  using rank1_collector_t = pressio4py::OdeCollectorWrapper<rom_rank1_state_t>;
  using rank2_collector_t = pressio4py::OdeCollectorWrapper<rom_rank2_state_t>;
  using rank3_collector_t = pressio4py::OdeCollectorWrapper<rom_rank3_state_t>;

  // ******************************************************
  // *** GALERKIN explicit Rank-1 state, rank-2 decoder ***
  pressio4py::bindGalerkinExplicitProbs
    <explicit_12_galerkinproblems>::template bind<
      rom_rank1_state_t, ::pressio4py::scalar_t, rank1_collector_t>(galerkinModule);
  pressio4py::bindGalerkinExplicitProbs
    <explicit_12_galerkinproblems>::template bind<
      rom_rank1_state_t, ::pressio4py::scalar_t>(galerkinModule);

  // ******************************************************
  // *** GALERKIN explicit Rank-2 state, rank-3 decoder ***
  pressio4py::bindGalerkinExplicitProbs
    <explicit_23_galerkinproblems>::template bind<
      rom_rank2_state_t, ::pressio4py::scalar_t, rank2_collector_t>(galerkinModule);
  pressio4py::bindGalerkinExplicitProbs
    <explicit_23_galerkinproblems>::template bind<
      rom_rank2_state_t, ::pressio4py::scalar_t>(galerkinModule);

  // ******************************************************
  // GALERKIN implicit time stepping with collector object
  pressio4py::bindSingleSolverWithMultipleGalerkinProblems
    <newraph_solver_t, implicit_galerkinproblems>::template bind<
      rom_rank1_state_t, ::pressio4py::scalar_t, rank1_collector_t>(galerkinModule);

  // ******************************************************
  // implicit time stepping without collector object
  pressio4py::bindSingleSolverWithMultipleGalerkinProblems
    <newraph_solver_t, implicit_galerkinproblems>::template bind<
      rom_rank1_state_t, ::pressio4py::scalar_t>(galerkinModule);

  // ****************
  // *** lspg ***
  // for lspg, we have MANY solver choices and MANY problem types,
  // so writing explicitly the binding for solving all these is verbose.
  // So we use some metaprogramming to facilitate binding each problem to each problem.
  pressio4py::bindLspgProbsWithMultipleSolvers
    <steady_lspgproblems, least_squares_solvers>::template bind<
      rom_rank1_state_t>(lspgModule);
  // unsteady with collector object
  pressio4py::bindLspgProbsWithMultipleSolvers
    <unsteady_lspgproblems, least_squares_solvers>::template bind<
      rom_rank1_state_t, rank1_collector_t>(lspgModule);
  // unsteady without collector object
  pressio4py::bindLspgProbsWithMultipleSolvers
    <unsteady_lspgproblems, least_squares_solvers>::template bind<
      rom_rank1_state_t>(lspgModule);

}
#endif
