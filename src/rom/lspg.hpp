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

#ifndef PRESSIO4PY_PYBINDINGS_LSPG_HPP_
#define PRESSIO4PY_PYBINDINGS_LSPG_HPP_

namespace pressio4py{

namespace impl{

// the problemid is used to choose among: default, masked and preconditioned
// so 0 = default, 1=masked, 2==preconditioned
template <typename mytypes, typename ode_tag, int problemid>
struct UnsteadyLSPGProblemBinder
{
  using scalar_t	= typename mytypes::scalar_t;
  using fom_t		= typename mytypes::fom_t;
  using rom_native_state_t	= typename mytypes::rom_native_state_t;
  using fom_native_state_t	= typename mytypes::fom_native_state_t;
  using rom_state_t	= typename mytypes::rom_state_t;
  using decoder_t	= typename mytypes::decoder_t;

  static_assert(problemid==0, "currently only supporting default LSPG");
  using lspg_problem_type = typename std::conditional<
    problemid==0,
    typename pressio::rom::lspg::composeDefaultProblem<ode_tag, fom_t, rom_state_t, decoder_t>::type,
    void>::type;

  using lspg_stepper_t	= typename lspg_problem_type::lspg_stepper_t;
  using residual_policy_t	= typename lspg_problem_type::lspg_residual_policy_t;
  using jacobian_policy_t	= typename lspg_problem_type::lspg_jacobian_policy_t;

  UnsteadyLSPGProblemBinder() = default;

  void bind(pybind11::module & m,
	    const std::string appendToStepper,
	    const std::string appendToProblem) const
  {
    const auto stepperPythonName = "Stepper"+appendToStepper;
    const auto problemPythonName = "Problem"+appendToProblem;

    // concrete LSPG stepper binding (need this because inside python we extract it
    // from the problem object to pass to the advancer
    pybind11::class_<lspg_stepper_t> stepper(m, stepperPythonName.c_str());
    stepper.def(pybind11::init<const rom_state_t &,
		const fom_t &,
		const residual_policy_t &,
		const jacobian_policy_t &>());

    // concrete LSPG problem binding: need this because is what we use to extract stepper
    pybind11::class_<lspg_problem_type> problem(m, problemPythonName.c_str());
    problem.def(pybind11::init<const fom_t &, const fom_native_state_t &,
		const decoder_t &, rom_native_state_t &, scalar_t>());
    problem.def("getFomStateReconstructor", &lspg_problem_type::getFomStateReconstructorCRef,
		pybind11::return_value_policy::reference);
    problem.def("getStepper", &lspg_problem_type::getStepperRef,
		pybind11::return_value_policy::reference);
  }
};


// the problemid is used to choose among: default, masked, preconditioned
// so 0 = default, 1=masked, 2==preconditioned
template <typename mytypes, int problemid>
struct SteadyLSPGProblemBinder
{
  using scalar_t	= typename mytypes::scalar_t;
  using fom_t		= typename mytypes::fom_t;
  using rom_native_state_t	= typename mytypes::rom_native_state_t;
  using fom_native_state_t	= typename mytypes::fom_native_state_t;
  using rom_state_t	= typename mytypes::rom_state_t;
  using decoder_t	= typename mytypes::decoder_t;

  static_assert(problemid==0, "currently only supporting default LSPG");
  using lspg_problem_type = typename std::conditional<
    problemid==0,
    typename pressio::rom::lspg::composeDefaultProblem<fom_t, rom_state_t, decoder_t>::type,
    void>::type;

  using lspg_system_t		= typename lspg_problem_type::lspg_system_t;
  using residual_policy_t	= typename lspg_problem_type::lspg_residual_policy_t;
  using jacobian_policy_t	= typename lspg_problem_type::lspg_jacobian_policy_t;

  SteadyLSPGProblemBinder() = default;

  void bind(pybind11::module & m) const
  {
    // concrete LSPG system binding (need this because inside python we extract it
    // from the problem object to pass to the solver)
    pybind11::class_<lspg_system_t> system(m, "System");
    system.def(pybind11::init<const fom_t &, const residual_policy_t &, const jacobian_policy_t &>());

    // concrete LSPG problem
    pybind11::class_<lspg_problem_type> problem(m, "Problem");
    problem.def(pybind11::init<const fom_t &, const fom_native_state_t, const decoder_t &>());
    problem.def("getFomStateReconstructor", &lspg_problem_type::getFomStateReconstructorCRef,
		pybind11::return_value_policy::reference);
    problem.def("getSystem", &lspg_problem_type::getSystemRef,
		pybind11::return_value_policy::reference);
  }
};

}//end impl


template <typename mytypes>
struct LSPGBinder
{
  using scalar_t		= typename mytypes::scalar_t;
  using rom_native_state_t	= typename mytypes::rom_native_state_t;
  using hessian_t		= typename mytypes::hessian_t;

  // default steady
  using LSPGProblemBinder_t = impl::SteadyLSPGProblemBinder<mytypes, 0>;
  using lspg_steady_system_t = typename LSPGProblemBinder_t::lspg_system_t;

  // default unsteady
  using bdf1tag = pressio::ode::implicitmethods::Euler;
  using bdf1LSPGProblemBinder_t = impl::UnsteadyLSPGProblemBinder<mytypes, bdf1tag, 0>;
  using lspg_stepper_bdf1_t = typename bdf1LSPGProblemBinder_t::lspg_stepper_t;

  LSPGBinder(pybind11::module & lspgModule)
  {
    //--------------------------
    // *** steady problem ***
    //--------------------------
    pybind11::module lspgSteadyModule = lspgModule.def_submodule("steady");
    // default LSPG
    pybind11::module lspgSteadyDefaultModule = lspgSteadyModule.def_submodule("default");
    LSPGProblemBinder_t steadyBinder;
    steadyBinder.bind(lspgSteadyDefaultModule);

    //--------------------------
    // *** unsteady problem ***
    //--------------------------
    pybind11::module lspgUnsteadyModule = lspgModule.def_submodule("unsteady");

    // default LSPG
    pybind11::module lspgUnsteadyDefaultModule = lspgUnsteadyModule.def_submodule("default");
    bdf1LSPGProblemBinder_t unsteadyBinder;
    unsteadyBinder.bind(lspgUnsteadyDefaultModule, "Euler", "Euler");
  }
};

}//end namespace
#endif
