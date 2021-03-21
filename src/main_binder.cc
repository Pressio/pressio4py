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
#include "./rom/wrappers/fom_discrete_time_wrapper.hpp"
#include "./rom/wrappers/lin_solver_wrapper.hpp"
#include "./rom/wrappers/qr_solver_wrapper.hpp"
#include "./rom/wrappers/ode_collector_wrapper.hpp"
#include "./rom/wrappers/nonlin_ls_weighting_wrapper.hpp"
#include "./rom/wrappers/masker_wrapper.hpp"

#include "logger.hpp"
#include "./rom/decoder.hpp"
#include "./rom/fomreconstructor.hpp"
#include "./rom/galerkin.hpp"
#include "./rom/lspg.hpp"
#include "./rom/wls.hpp"
#include "./rom/nonlinear_solvers.hpp"

#include "./galerkin_solve_problem_api_bind.hpp"
#include "./lspg_solve_problem_api_bind.hpp"
#include "./wls_solve_problem_api_bind.hpp"

PYBIND11_MODULE(MODNAME, mParent)
{
  // bind logging functions
  pressio4py::bindLogger(mParent);

  //*****************
  // bind ROM
  //*****************
  // the main rom module will work with the "standard" rank-1 state,
  // and create submodules to include functionalities for the rank-2,3 states
  pybind11::module romModule = mParent.def_submodule("rom");
  pybind11::module r2m = romModule.def_submodule("rank2state");
  pybind11::module r3m = romModule.def_submodule("rank3state");

  // *** decoder ***
  // <n,m>: n=state_rank, m=decoder_jacobian_rank
  pressio4py::bindDecoder<pressio4py::DecoderTypes<1,2>>(romModule, "Decoder");
  pressio4py::bindDecoder<pressio4py::DecoderTypes<2,2>>(r2m,	    "Decoder");
  pressio4py::bindDecoder<pressio4py::DecoderTypes<2,3>>(r2m,	    "MultiFieldDecoder");
  pressio4py::bindDecoder<pressio4py::DecoderTypes<3,3>>(r3m,	    "MultiFieldDecoder");

  // *** fom reconstructor ***
  pressio4py::bindFomReconstructor<pressio4py::DecoderTypes<1,2>>(romModule, "FomReconstructor");
  pressio4py::bindFomReconstructor<pressio4py::DecoderTypes<2,2>>(r2m,       "FomReconstructor");
  pressio4py::bindFomReconstructor<pressio4py::DecoderTypes<2,3>>(r2m,       "MultiFieldFomReconstructor");
  pressio4py::bindFomReconstructor<pressio4py::DecoderTypes<3,3>>(r3m,       "MultiFieldFomReconstructor");

  //--------------------------------
  // galerkin rom
  //--------------------------------
  pybind11::module galerkinModule           = romModule.def_submodule("galerkin");
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
  // this includes the implicit problems for continuous time api as well as
  // the problems types for the discrete time api
  using galerkin_implicit_binder_t  = pressio4py::rom::GalerkinBinderImplicit<pressio4py::GalerkinTypes<1,2>>;
  galerkin_implicit_binder_t::bind(galerkinModule);
  using implicit_galerkinproblems = typename galerkin_implicit_binder_t::problem_types;

  // *** newton-raphson solver ***
  using head_problem_t = typename std::tuple_element<0, implicit_galerkinproblems>::type;
  // we should make sure that all problems have same jacobian_t
  // but for implicit problems this should be true
  using implicit_galerkin_jacobian_t = typename head_problem_t::galerkin_jacobian_t;
  using linear_solver_nr_t = pressio4py::LinSolverWrapper<implicit_galerkin_jacobian_t>;
  using newraphbinder_t = pressio4py::solvers::NewtonRaphsonBinder<
    true, linear_solver_nr_t, implicit_galerkinproblems>;
  using newraph_solver_t = typename newraphbinder_t::nonlinear_solver_t;
  newraphbinder_t::bindClass(galerkinModule, "GalerkinNewtonRaphson");


  //---------------------
  //  lspg rom
  //---------------------
  pybind11::module lspgModule	= romModule.def_submodule("lspg");
  using lspg_binder_t		= pressio4py::rom::LSPGBinder<pressio4py::LspgTypes>;
  lspg_binder_t::bind(lspgModule);
  using lspgproblems		= typename lspg_binder_t::problem_types;
  using steady_lspgproblems	= typename lspg_binder_t::steady_problem_types;
  using unsteady_lspgproblems	= typename lspg_binder_t::unsteady_problem_types;

  // *** nonlin least squares solvers for LSPG ***
  using hessian_t	= typename pressio4py::LspgTypes::lsq_hessian_t;
  using jac_t           = typename pressio4py::LspgTypes::decoder_jac_t;
  using linear_solver_t = pressio4py::LinSolverWrapper<hessian_t>;
  using qr_solver_t     = pressio4py::QrSolverWrapper<jac_t>;
  using nlls_weigher_t  = pressio4py::NonLinLSWeightingWrapper;

  // *** GN RJ-API with normal equations ***
  using gnbinder_t = pressio4py::solvers::LeastSquaresNormalEqResJacApiBinder<
    true, linear_solver_t, lspgproblems>;
  using gn_solver_t = typename gnbinder_t::nonlinear_solver_t;
  gnbinder_t::bindClass(lspgModule, "LspgGaussNewton");

  // *** LM RJ-API with normal equations ***
  using lmbinder_t = pressio4py::solvers::LeastSquaresNormalEqResJacApiBinder<
    false, linear_solver_t, lspgproblems>;
  using lm_solver_t = typename lmbinder_t::nonlinear_solver_t;
  lmbinder_t::bindClass(lspgModule, "LspgLevenbergMarquardt");

  // *** GN RJ-API with QR ***
  using gnbinder_qr_t = pressio4py::solvers::LeastSquaresQRBinder<
    true, qr_solver_t, lspgproblems>;
  using gn_qr_solver_t = typename gnbinder_qr_t::nonlinear_solver_t;
  gnbinder_qr_t::bindClass(lspgModule, "LspgGaussNewtonQR");

  // *** weighted GN RJ-API with normal equations ***
  using wgnbinder_t = pressio4py::solvers::WeightedLeastSquaresNormalEqBinder<
    true, linear_solver_t,  nlls_weigher_t, lspgproblems>;
  using w_gn_solver_t = typename wgnbinder_t::nonlinear_solver_t;
  wgnbinder_t::bindClass(lspgModule, "LspgWeightedGaussNewton");

  // *** IRW GN RJ-API with normal equations ***
  using irwgnbinder_t = pressio4py::solvers::IrwLeastSquaresNormalEqBinder<
    linear_solver_t,  lspgproblems>;
  using irwgn_solver_t = typename irwgnbinder_t::nonlinear_solver_t;
  irwgnbinder_t::bindClass(lspgModule, "LspgIrwGaussNewton");

  // create tuple with all LSPG least-squares solver types
  using lspg_solvers = std::tuple<
    gn_solver_t, irwgn_solver_t, w_gn_solver_t,
    gn_qr_solver_t, lm_solver_t>;

  //----------------------------------------------------
  //  wls rom (still experimental)
  //----------------------------------------------------
  pybind11::module expModule	= romModule.def_submodule("exp");
  pybind11::module wlsModule	= expModule.def_submodule("wls");
  using wls_binder_t = pressio4py::rom::WLSBinder<pressio4py::WlsTypes>;
  wls_binder_t::bind(wlsModule);
  using wls_systems = typename wls_binder_t::system_types;

  using wlshessian_t	= typename pressio4py::WlsTypes::lsq_hessian_t;
  using wlslinear_solver_t = pressio4py::LinSolverWrapper<wlshessian_t>;

  // *** GN ***
  using wlsgnbinder_t = pressio4py::solvers::LeastSquaresNormalEqHessGrapApiBinder<
    true, wlslinear_solver_t, wls_systems>;
  using wlsgn_solver_t = typename wlsgnbinder_t::nonlinear_solver_t;
  wlsgnbinder_t::bindClass(wlsModule, "WlsGaussNewton");

  // *** LM ***
  using wlslmbinder_t = pressio4py::solvers::LeastSquaresNormalEqHessGrapApiBinder<
    false, wlslinear_solver_t, wls_systems>;
  using wlslm_solver_t = typename wlslmbinder_t::nonlinear_solver_t;
  wlslmbinder_t::bindClass(wlsModule, "WlsLevenbergMarquardt");

  // create tuple with all Wls solvers types
  using wls_solvers = std::tuple<
    wlsgn_solver_t, wlslm_solver_t>;

  //******************
  // solvers module
  //*****************
  pybind11::module mSolver = mParent.def_submodule("solvers");
  pressio4py::solvers::bindUpdatingEnums(mSolver);
  pressio4py::solvers::bindStoppingEnums(mSolver);

  // contains createFunctions to instantiate solver objects
  // specified above for Galerkin, lspg, etc
  gnbinder_t::bindCreate(mSolver);
  lmbinder_t::bindCreate(mSolver);
  gnbinder_qr_t::bindCreate(mSolver);
  wgnbinder_t::bindCreate(mSolver);
  irwgnbinder_t::bindCreate(mSolver);
  wlsgnbinder_t::bindCreate(mSolver);
  wlslmbinder_t::bindCreate(mSolver);
  newraphbinder_t::bindCreate(mSolver);

  //---------------------------------------------------------------------
  //---------------------------------------------------------------------
  // expose api to solve galerkin, lspg and wls problems
  //
  // one way to do this would be to write explicitly the binding
  // code for solving all problems with all possible solver
  // combinations, but this is not practical.
  // use metaprogramming to simplify the binding code generation.
  //---------------------------------------------------------------------
  //---------------------------------------------------------------------
  using rom_rank1_state_t  = typename pressio4py::StateTypes<1>::rom_state_t;
  using rom_rank2_state_t  = typename pressio4py::StateTypes<2>::rom_state_t;
  using rom_rank3_state_t  = typename pressio4py::StateTypes<3>::rom_state_t;

  // collector used to monitor the rom state
  using rank1_collector_t = pressio4py::OdeCollectorWrapper<rom_rank1_state_t>;
  using rank2_collector_t = pressio4py::OdeCollectorWrapper<rom_rank2_state_t>;
  using rank3_collector_t = pressio4py::OdeCollectorWrapper<rom_rank3_state_t>;

  // *** GALERKIN explicit Rank-1 state, rank-2 decoder ***
  // without collector
  pressio4py::bindGalerkinExplicitProbs
    <explicit_12_galerkinproblems>::template bind<
      rom_rank1_state_t, ::pressio4py::scalar_t>(galerkinModule);
  // with collector
  pressio4py::bindGalerkinExplicitProbs
    <explicit_12_galerkinproblems>::template bind<
      rom_rank1_state_t, ::pressio4py::scalar_t, rank1_collector_t>(galerkinModule);

  // *** GALERKIN explicit Rank-2 state, rank-3 decoder ***
  // without collector
  pressio4py::bindGalerkinExplicitProbs
    <explicit_23_galerkinproblems>::template bind<
      rom_rank2_state_t, ::pressio4py::scalar_t>(galerkinModule);
  // with collector
  pressio4py::bindGalerkinExplicitProbs
    <explicit_23_galerkinproblems>::template bind<
      rom_rank2_state_t, ::pressio4py::scalar_t, rank2_collector_t>(galerkinModule);

  // GALERKIN implicit time stepping with collector object
  pressio4py::bindSingleSolverWithMultipleGalerkinProblems
    <newraph_solver_t, implicit_galerkinproblems>::template bind<
      rom_rank1_state_t, ::pressio4py::scalar_t, rank1_collector_t>(galerkinModule);

  // implicit time stepping without collector object
  pressio4py::bindSingleSolverWithMultipleGalerkinProblems
    <newraph_solver_t, implicit_galerkinproblems>::template bind<
      rom_rank1_state_t, ::pressio4py::scalar_t>(galerkinModule);

  // *** lspg ***
  pressio4py::bindLspgProbsWithMultipleSolvers
    <steady_lspgproblems, lspg_solvers>::template bind<
      rom_rank1_state_t>(lspgModule);
  // unsteady with collector object
  pressio4py::bindLspgProbsWithMultipleSolvers
    <unsteady_lspgproblems, lspg_solvers>::template bind<
      rom_rank1_state_t, rank1_collector_t>(lspgModule);
  // unsteady without collector object
  pressio4py::bindLspgProbsWithMultipleSolvers
    <unsteady_lspgproblems, lspg_solvers>::template bind<
      rom_rank1_state_t>(lspgModule);

  // *** wls ***
  pressio4py::bindWlsSystemsWithMultipleSolvers
    <wls_systems, wls_solvers>::template bind<
      pressio4py::WlsTypes::rom_state_t>(wlsModule);

}
#endif
