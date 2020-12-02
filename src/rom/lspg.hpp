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

namespace pressio4py{ namespace rom{ namespace impl{

// the problemid is used to choose among:
// so 0 = default
template <typename mytypes, int problemid>
struct SteadyLSPGProblemBinder
{
  using scalar_t	   = typename mytypes::scalar_t;
  using fom_native_state_t = typename mytypes::fom_native_state_t;
  using rom_native_state_t = typename mytypes::rom_native_state_t;
  using rom_state_t	   = typename mytypes::rom_state_t;
  using decoder_t	   = typename mytypes::decoder_t;
  using decoder_native_jac_t = typename mytypes::decoder_native_jac_t;

  using sys_wrapper_t = pressio4py::rom::FomWrapperSteadyState<
    scalar_t, fom_native_state_t, fom_native_state_t, decoder_native_jac_t>;

  static_assert(problemid==0, "currently only supporting default LSPG");
  using lspg_problem_t = typename std::conditional<
    problemid==0,
    typename pressio::rom::lspg::impl::composeDefaultProblem<
      sys_wrapper_t, decoder_t, rom_state_t>::type,
    void
    >::type;

  using lspg_system_t		= typename lspg_problem_t::system_t;
  using residual_policy_t	= typename lspg_problem_t::residual_policy_t;
  using jacobian_policy_t	= typename lspg_problem_t::jacobian_policy_t;

  SteadyLSPGProblemBinder() = default;

  static void bind(pybind11::module & m)
  {
    // concrete LSPG system binding (need this because inside python we extract it
    // from the problem object to pass to the solver)
    pybind11::class_<lspg_system_t> system(m, "System");
    system.def(pybind11::init<
	       sys_wrapper_t,
	       const residual_policy_t &,
	       const jacobian_policy_t &>());

    // concrete LSPG problem
    pybind11::class_<lspg_problem_t> problem(m, "Problem");
    problem.def(pybind11::init<
		pybind11::object,
		const decoder_t &,
		const rom_native_state_t &,
		const fom_native_state_t>());
    problem.def("fomStateReconstructor",
		&lspg_problem_t::fomStateReconstructorCRef,
		pybind11::return_value_policy::reference);
    problem.def("system",
		&lspg_problem_t::systemRef,
		pybind11::return_value_policy::reference);
  }
};


// the problemid is used to choose among
// so 0 = default, 1=hypred
template <typename mytypes, typename ode_tag, int problemid>
struct UnsteadyLSPGProblemBinder
{
  using scalar_t	   = typename mytypes::scalar_t;
  using rom_native_state_t = typename mytypes::rom_native_state_t;
  using fom_native_state_t = typename mytypes::fom_native_state_t;
  using rom_state_t	   = typename mytypes::rom_state_t;
  using decoder_t	   = typename mytypes::decoder_t;
  using decoder_native_jac_t = typename mytypes::decoder_native_jac_t;

  using sys_wrapper_t = pressio4py::rom::FomWrapperCTimeWithApplyJac<
    scalar_t, fom_native_state_t, fom_native_state_t, decoder_native_jac_t>;

  using lspg_problem_t =
    typename std::conditional<
    problemid==0,
    typename pressio::rom::lspg::impl::composeDefaultProblem_t<
      ode_tag, sys_wrapper_t, decoder_t, rom_state_t>,
    typename std::conditional<
      problemid==1,
      typename pressio::rom::lspg::impl::composeHyperReducedProblem_t<
	ode_tag, sys_wrapper_t, decoder_t, rom_state_t,
	// we use a py_f_arr to store indices for sample to stencil mapping
	pressio::containers::Vector<typename mytypes::py_f_arr>>,
      void
      >::type
    >::type;

  using lspg_stepper_t	  = typename lspg_problem_t::stepper_t;
  using aux_lspg_stepper_t= typename lspg_problem_t::aux_stepper_t;
  using residual_policy_t = typename lspg_problem_t::residual_policy_t;
  using jacobian_policy_t = typename lspg_problem_t::jacobian_policy_t;

  // constructor for default lspg problem
  template<typename T, int _problemid = problemid>
  static typename std::enable_if<_problemid==0>::type
  bindProblemConstructor(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pybind11::object,
		const decoder_t &,
		const rom_native_state_t &,
		const fom_native_state_t &>());
  }

  // constructor for hyper-reduced lspg problem
  template<typename T, int _problemid = problemid>
  static typename std::enable_if<_problemid==1>::type
  bindProblemConstructor(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pybind11::object,
		const decoder_t &,
		const rom_native_state_t &,
		const fom_native_state_t &,
		typename mytypes::py_f_arr>());
  }

  static void bind(pybind11::module & m,
		   const std::string appendToStepperName,
		   const std::string appendToProblemName)
  {
    const auto stepperPythonName = "Stepper"+appendToStepperName;
    const auto problemPythonName = "Problem"+appendToProblemName;

    // // concrete LSPG stepper binding (need this because inside python we extract it
    // // from the problem object to pass to the advancer
    // pybind11::class_<lspg_stepper_t> stepper(m, stepperPythonName.c_str());
    // stepper.def(pybind11::init<
    // 		const rom_state_t &,
    // 		const sys_wrapper_t &,
    // 		const residual_policy_t &,
    // 		const jacobian_policy_t &>());

    // concrete LSPG problem binding: need this because is what we use to extract stepper
    pybind11::class_<lspg_problem_t> problem(m, problemPythonName.c_str());
    bindProblemConstructor(problem);
    problem.def("fomStateReconstructor",
		&lspg_problem_t::fomStateReconstructorCRef,
		pybind11::return_value_policy::reference);
    // problem.def("stepper",
    // 		&lspg_problem_t::stepperRef,
    // 		pybind11::return_value_policy::reference);
  }
};

}//end impl


template <typename mytypes>
struct LSPGBinder
{
  using scalar_t		= typename mytypes::scalar_t;
  using rom_native_state_t	= typename mytypes::rom_native_state_t;

  // steady
  using LSPGProblemBinder_t = impl::SteadyLSPGProblemBinder<mytypes, 0>;
  using lspg_steady_problem_t = typename LSPGProblemBinder_t::lspg_problem_t;
  using lspg_steady_system_t  = typename LSPGProblemBinder_t::lspg_system_t;

  //------------------
  // unsteady: bdf1
  //------------------
  using bdf1tag = pressio::ode::implicitmethods::Euler;
  // default
  using de_bdf1LSPGProblemBinder_t = impl::UnsteadyLSPGProblemBinder<mytypes, bdf1tag, 0>;
  using de_lspg_problem_bdf1_t	   = typename de_bdf1LSPGProblemBinder_t::lspg_problem_t;
  using de_lspg_stepper_bdf1_t     = typename de_bdf1LSPGProblemBinder_t::lspg_stepper_t;
  // hyper-reduced
  using hr_bdf1LSPGProblemBinder_t = impl::UnsteadyLSPGProblemBinder<mytypes, bdf1tag, 1>;
  using hr_lspg_problem_bdf1_t	   = typename hr_bdf1LSPGProblemBinder_t::lspg_problem_t;
  using hr_lspg_stepper_bdf1_t     = typename hr_bdf1LSPGProblemBinder_t::lspg_stepper_t;

  //------------------
  // unsteady: bdf2
  //------------------
  using bdf2tag = pressio::ode::implicitmethods::BDF2;
  // default
  using de_bdf2LSPGProblemBinder_t = impl::UnsteadyLSPGProblemBinder<mytypes, bdf2tag, 0>;
  using de_lspg_problem_bdf2_t	   = typename de_bdf2LSPGProblemBinder_t::lspg_problem_t;
  using de_lspg_stepper_bdf2_t     = typename de_bdf2LSPGProblemBinder_t::lspg_stepper_t;
  // hyper-reduced
  using hr_bdf2LSPGProblemBinder_t = impl::UnsteadyLSPGProblemBinder<mytypes, bdf2tag, 1>;
  using hr_lspg_problem_bdf2_t	   = typename hr_bdf2LSPGProblemBinder_t::lspg_problem_t;
  using hr_lspg_stepper_bdf2_t     = typename hr_bdf2LSPGProblemBinder_t::lspg_stepper_t;

  static void bind(pybind11::module & lspgModule)
  {
    //--------------------------
    // *** steady problem ***
    //--------------------------
    {
      pybind11::module lspgSteadyModule = lspgModule.def_submodule("steady");
      // default
      pybind11::module lspgSteadyDefaultModule = lspgSteadyModule.def_submodule("default");
      LSPGProblemBinder_t::bind(lspgSteadyDefaultModule);
    }

    //--------------------------
    // *** unsteady problem ***
    //--------------------------
    pybind11::module lspgUnsteadyModule = lspgModule.def_submodule("unsteady");
    {
      // default LSPG
      pybind11::module thismodule = lspgUnsteadyModule.def_submodule("default");
      de_bdf1LSPGProblemBinder_t::bind(thismodule, "Euler", "Euler");
      de_bdf2LSPGProblemBinder_t::bind(thismodule, "BDF2", "BDF2");
    }
    {
      // hyper-reduced LSPG
      pybind11::module thismodule = lspgUnsteadyModule.def_submodule("hyperreduced");
      hr_bdf1LSPGProblemBinder_t::bind(thismodule, "Euler", "Euler");
      hr_bdf2LSPGProblemBinder_t::bind(thismodule, "BDF2", "BDF2");
    }
  }
};

}}//end namespace pressio4py::rom
#endif
