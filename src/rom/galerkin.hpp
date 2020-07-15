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

namespace pressio4py{ namespace impl{

template <typename mytypes, typename ode_tag>
void createGalerkinBindingsImpl(pybind11::module & m,
				const std::string stepperName,
				const std::string problemName,
				const std::string advancerName)
{
  using scalar_t	   = typename mytypes::scalar_t;
  using fom_t		   = typename mytypes::fom_t;
  using ops_t		   = typename mytypes::ops_t;
  using rom_native_state_t = typename mytypes::rom_native_state_t;
  using fom_native_state_t = typename mytypes::fom_native_state_t;
  using rom_state_t	   = typename mytypes::rom_state_t;
  using decoder_t	   = typename mytypes::decoder_t;

  using gal_problem_t		= typename pressio::rom::galerkin::composeDefaultProblem<
    ode_tag, fom_t, rom_state_t, decoder_t>::type;
  using res_pol_t		= typename gal_problem_t::residual_policy_t;
  using galerkin_stepper_t	= typename gal_problem_t::stepper_t;

  // stepper
  pybind11::class_<galerkin_stepper_t>(m, stepperName.c_str())
    .def(pybind11::init<const rom_state_t &, const fom_t &, const res_pol_t &>());

  pybind11::class_<gal_problem_t>(m, problemName.c_str())
    .def(pybind11::init<const fom_t &, fom_native_state_t, const decoder_t &, rom_native_state_t, scalar_t>())
    .def("getStepper", &gal_problem_t::getStepperRef,
	 pybind11::return_value_policy::reference)
    .def("getFomStateReconstructor", &gal_problem_t::getFomStateReconstructorCRef,
	 pybind11::return_value_policy::reference);

  // integrator
  m.def(advancerName.c_str(),
	&::pressio::ode::advanceNSteps<galerkin_stepper_t, rom_native_state_t, scalar_t>,
	"Advance N Steps");
}
}// end impl

template <typename mytypes>
void createGalerkinBindings(pybind11::module & m)
{
  using scalar_t	   = typename mytypes::scalar_t;
  using fom_t		   = typename mytypes::fom_t;
  using ops_t		   = typename mytypes::ops_t;
  using rom_native_state_t = typename mytypes::rom_native_state_t;
  using fom_native_state_t = typename mytypes::fom_native_state_t;
  using rom_state_t	   = typename mytypes::rom_state_t;
  using decoder_t	   = typename mytypes::decoder_t;

  using tag1 = pressio::ode::explicitmethods::Euler;
  impl::createGalerkinBindingsImpl<mytypes,tag1>(m, "StepperEuler", "ProblemEuler", "advanceNStepsEuler");

  using tag2 = pressio::ode::explicitmethods::RungeKutta4;
  impl::createGalerkinBindingsImpl<mytypes,tag2>(m, "StepperRK4", "ProblemRK4", "advanceNStepsRK4");
}

}//namespace pressio4py
#endif
