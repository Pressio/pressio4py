/*
//@HEADER
// ************************************************************************
//
// galerkin.hpp
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

#ifndef PRESSIO4PY_PYBINDINGS_GALERKIN_HPP_
#define PRESSIO4PY_PYBINDINGS_GALERKIN_HPP_

#include "pressio_rom.hpp"
#include "types.hpp"

PYBIND11_MODULE(pressio4pyGalerkin, m) {

  using mytypes		= DefaultGalerkinTypes;
  using scalar_t	= typename mytypes::scalar_t;
  using fom_t		= typename mytypes::fom_t;
  using ops_t		= typename mytypes::ops_t;

  using rom_state_t	= typename mytypes::rom_state_t;
  using fom_state_t	= typename mytypes::fom_state_t;
  using decoder_jac_t	= typename mytypes::decoder_jac_t;

  // --------------------------------------------------------------------
  // decoder
  // --------------------------------------------------------------------
  using decoder_t	= ::pressio::rom::PyLinearDecoder<decoder_jac_t, ops_t, rom_state_t, fom_state_t>;
  using decoder_base_t = typename decoder_t::base_t;

  // base decoder class
  pybind11::class_<decoder_base_t>(m, "DecoderBase")
    .def("applyMapping", &decoder_base_t::template applyMapping<rom_state_t>);

  pybind11::class_<decoder_t, decoder_base_t>(m, "LinearDecoder")
    .def(pybind11::init< const decoder_jac_t &>());


  //--------------------------------------------------------
  // Euler Galerkin problem
  //--------------------------------------------------------
  {
    using ode_tag = pressio::ode::explicitmethods::Euler;
    using pressio::rom::galerkin::DefaultProblemType;
    using galerkin_t = DefaultProblemType<ode_tag, rom_state_t, fom_t, decoder_t, ops_t>;

    using galerkin_problem_gen	= pressio::rom::galerkin::ProblemGenerator<galerkin_t>;
    using res_pol_t		= typename galerkin_problem_gen::galerkin_residual_policy_t;
    using galerkin_stepper_t	= typename galerkin_problem_gen::galerkin_stepper_t;

    // stepper
    pybind11::class_<galerkin_stepper_t>(m, "StepperEuler")
      .def(pybind11::init<const rom_state_t &, const fom_t &, const res_pol_t &>());

    pybind11::class_<galerkin_problem_gen>(m, "ProblemEuler")
      .def(pybind11::init<const fom_t &, const fom_state_t &, const decoder_t &,
    			  rom_state_t &, scalar_t>())
      .def("getStepper", &galerkin_problem_gen::getStepperRef);

    // integrator
    m.def("integrateNStepsEuler",
    	  &::pressio::ode::integrateNSteps<galerkin_stepper_t, rom_state_t, scalar_t>,
    	  "Integrate N Steps");
  }

  //--------------------------------------------------------
  // RK4 Galerkin problem
  //--------------------------------------------------------
  {
    using ode_tag = pressio::ode::explicitmethods::RungeKutta4;
    using pressio::rom::galerkin::DefaultProblemType;
    using galerkin_t = DefaultProblemType<ode_tag, rom_state_t, fom_t, decoder_t, ops_t>;

    using galerkin_problem_gen	= pressio::rom::galerkin::ProblemGenerator<galerkin_t>;
    using res_pol_t		= typename galerkin_problem_gen::galerkin_residual_policy_t;
    using galerkin_stepper_t	= typename galerkin_problem_gen::galerkin_stepper_t;

    // stepper
    pybind11::class_<galerkin_stepper_t>(m, "StepperRK4")
      .def(pybind11::init<const rom_state_t &, const fom_t &, const res_pol_t &>());

    pybind11::class_<galerkin_problem_gen>(m, "ProblemRK4")
      .def(pybind11::init<const fom_t &, const fom_state_t &, const decoder_t &,
  			  rom_state_t &, scalar_t>())
      .def("getStepper", &galerkin_problem_gen::getStepperRef);

    // integrator
    m.def("integrateNStepsRK4",
  	  &::pressio::ode::integrateNSteps<galerkin_stepper_t, rom_state_t, scalar_t>,
  	  "Integrate N Steps");
  }

}
#endif
