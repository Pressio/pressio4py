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
#include "types.hpp"
#include "./rom/decoder.hpp"
#include "./rom/fomreconstructor.hpp"
#include "./rom/fom_wrappers.hpp"
#include "./rom/galerkin.hpp"
#include "./rom/lspg.hpp"
#include "./rom/nonlinear_solvers.hpp"

PYBIND11_MODULE(pressio4py, mParent)
{
  pybind11::module m1 = mParent.def_submodule("rom");

  using scalar_t	   = typename pressio4py::ROMTypes::scalar_t;
  using rom_native_state_t = typename pressio4py::ROMTypes::rom_native_state_t;

  // decoder bindings
  pressio4py::createDecoderBindings<pressio4py::ROMTypes>(m1);

  // fom reconstructor bindings
  pressio4py::createFomReconstructorBindings<pressio4py::ROMTypes>(m1);

  // galerkin
  pybind11::module m2	  = m1.def_submodule("galerkin");
  using galerkin_binder_t = pressio4py::rom::GalerkinBinder<pressio4py::ROMTypes>;
  galerkin_binder_t galBinder(m2);

  // lspg
  pybind11::module m3	     = m1.def_submodule("lspg");
  using lspg_binder_t	     = pressio4py::rom::LSPGBinder<pressio4py::ROMTypes>;
  using lspg_steady_system_t = typename lspg_binder_t::lspg_steady_system_t;
  using lspg_stepper_bdf1_t  = typename lspg_binder_t::lspg_stepper_bdf1_t;
  lspg_binder_t lspgBinder(m3);

  //-------------------------------------
  // *** nonlinear l-squares solvers ***
  //-------------------------------------
  pybind11::module mSolver = mParent.def_submodule("solvers");
  pressio4py::solvers::bindUpdatingEnums(mSolver);
  pressio4py::solvers::bindStoppingEnums(mSolver);

  // GN with normal equations
  using hessian_t   = typename pressio4py::ROMTypes::hessian_t;
  using gnbinder_t = pressio4py::solvers::LeastSquaresNormalEqBinder<
    true, lspg_steady_system_t, lspg_stepper_bdf1_t, hessian_t, rom_native_state_t>;
  using gn_solver_t = typename gnbinder_t::nonlinear_solver_t;
  gnbinder_t::bind(mSolver, "GaussNewton");

  // LM with normal equations
  using lmbinder_t = pressio4py::solvers::LeastSquaresNormalEqBinder<
    false, lspg_steady_system_t, lspg_stepper_bdf1_t, hessian_t, rom_native_state_t>;
  using lm_solver_t = typename lmbinder_t::nonlinear_solver_t;
  lmbinder_t::bind(mSolver, "LevenbergMarquardt");

  //-------------------------------------
  // ode::advance
  //-------------------------------------
  pybind11::module mOde = mParent.def_submodule("ode");
  mOde.def("advanceNSteps", // for Galerkin Euler
	   &::pressio::ode::advanceNSteps<
	   typename galerkin_binder_t::stepper_euler_t, rom_native_state_t, scalar_t>);
  mOde.def("advanceNSteps", // for Galerkin RK4
	   &::pressio::ode::advanceNSteps<
	   typename galerkin_binder_t::stepper_rk4_t, rom_native_state_t, scalar_t>);

  mOde.def("advanceNSteps", // unsteady LSPG with GN
	   &pressio::ode::advanceNSteps<
	   lspg_stepper_bdf1_t, rom_native_state_t, scalar_t, gn_solver_t>);
  mOde.def("advanceNSteps", // unsteady LSPG with LM
	   &pressio::ode::advanceNSteps<
	   lspg_stepper_bdf1_t, rom_native_state_t, scalar_t, lm_solver_t>);

}

#endif
