/*
//@HEADER
// ************************************************************************
//
// lspg.hpp
//                     		  Pressio
//                         Copyright 2019
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

/*
  STEADY LSPG

  the problemid is used to choose the subcase:
  - default = 0
  - masked = 2
*/
template <typename mytypes, int problemid>
struct SteadyLSPGProblemBinder
{
  using fom_native_state_t = typename mytypes::fom_native_state_t;
  using rom_native_state_t = typename mytypes::rom_native_state_t;
  using rom_state_t	   = typename mytypes::rom_state_t;
  using decoder_t	   = typename mytypes::decoder_t;
  using decoder_native_jac_t = typename mytypes::decoder_native_jac_t;

  using sys_wrapper_t = pressio4py::rom::FomWrapperSteadyState<
    ::pressio4py::scalar_t, fom_native_state_t,
    fom_native_state_t, decoder_native_jac_t>;

  using masker_wrapper_t = pressio4py::rom::MaskerWrapper<::pressio4py::scalar_t>;

  using lspg_problem_t =
    typename std::conditional<
    problemid==0,
    pressio::rom::lspg::impl::composeDefaultProblem_t<
      sys_wrapper_t, decoder_t, rom_state_t>,
    typename std::conditional<
      problemid==2,
      pressio::rom::lspg::impl::composeMaskedProblem_t<
	sys_wrapper_t, decoder_t, rom_state_t, masker_wrapper_t>,
      void
      >::type
    >::type;

  // constructor for default lspg problem
  template<typename T, int _problemid = problemid>
  static typename std::enable_if<_problemid==0>::type
  bindProblemConstructor(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pybind11::object,
		decoder_t &,
		const rom_native_state_t &,
		const fom_native_state_t>());
  }

  // constructor for masked lspg problem
  template<typename T, int _problemid = problemid>
  static typename std::enable_if<_problemid==2>::type
  bindProblemConstructor(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pybind11::object,
		decoder_t &,
		const rom_native_state_t &,
		const fom_native_state_t &,
		pybind11::object>());
  }

  static void bind(pybind11::module & m)
  {
    pybind11::class_<lspg_problem_t> problem(m, "Problem");
    bindProblemConstructor(problem);
    problem.def("fomStateReconstructor",
		&lspg_problem_t::fomStateReconstructorCRef,
		pybind11::return_value_policy::reference);
  }
};


/*
  UNSTEADY LSPG

  the problemid is used to choose the subcase:
  - default = 0
  - hyper-reduced = 1
  - masked = 2
*/
template <typename mytypes, typename ode_tag, int problemid>
struct UnsteadyLSPGProblemBinder
{
  using rom_native_state_t = typename mytypes::rom_native_state_t;
  using fom_native_state_t = typename mytypes::fom_native_state_t;
  using rom_state_t	   = typename mytypes::rom_state_t;
  using decoder_t	   = typename mytypes::decoder_t;
  using decoder_native_jac_t = typename mytypes::decoder_native_jac_t;

  using sys_wrapper_t = pressio4py::rom::FomWrapperCTimeWithApplyJac<
    ::pressio4py::scalar_t, fom_native_state_t, fom_native_state_t, decoder_native_jac_t>;

  // only used when problemid==2
  using masker_wrapper_t = pressio4py::rom::MaskerWrapper<::pressio4py::scalar_t>;

  using lspg_problem_t =
    typename std::conditional<
    problemid==0,
    pressio::rom::lspg::impl::composeDefaultProblem_t<
      ode_tag, sys_wrapper_t, decoder_t, rom_state_t>,
    typename std::conditional<
      problemid==1,
      pressio::rom::lspg::impl::composeHyperReducedProblem_t<
	ode_tag, sys_wrapper_t, decoder_t, rom_state_t,
	// we use a py_f_arr to store indices for sample to stencil mapping
	pressio::containers::Tensor<1, pressio4py::py_f_arr>>,
      typename std::conditional<
	problemid==2,
	pressio::rom::lspg::impl::composeMaskedProblem_t<
	  ode_tag, sys_wrapper_t, decoder_t, rom_state_t, masker_wrapper_t>,
	void
	>::type
      >::type
    >::type;

  // constructor for default lspg problem
  template<typename T, int _problemid = problemid>
  static typename std::enable_if<_problemid==0>::type
  bindProblemConstructor(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pybind11::object,
		decoder_t &,
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
		decoder_t &,
		const rom_native_state_t &,
		const fom_native_state_t &,
		pressio4py::py_f_arr>());
  }

  // constructor for masked lspg problem
  template<typename T, int _problemid = problemid>
  static typename std::enable_if<_problemid==2>::type
  bindProblemConstructor(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pybind11::object,
		decoder_t &,
		const rom_native_state_t &,
		const fom_native_state_t &,
		pybind11::object>());
  }

  static void bind(pybind11::module & m, const std::string appendToProblemName)
  {
    const auto problemPythonName = "Problem"+appendToProblemName;
    pybind11::class_<lspg_problem_t> problem(m, problemPythonName.c_str());
    bindProblemConstructor(problem);
    problem.def("fomStateReconstructor",
		&lspg_problem_t::fomStateReconstructorCRef,
		pybind11::return_value_policy::reference);
  }
};

//--------------------------
}//end namespace impl
//--------------------------

template <typename mytypes>
struct LSPGBinder
{
  //------------------
  // steady default
  //------------------
  using de_steady_binder_t  = impl::SteadyLSPGProblemBinder<mytypes, 0>;
  using de_steady_problem_t = typename de_steady_binder_t::lspg_problem_t;
  //------------------
  // steady masked
  //------------------
  using ma_steady_binder_t  = impl::SteadyLSPGProblemBinder<mytypes, 2>;
  using ma_steady_problem_t = typename ma_steady_binder_t::lspg_problem_t;

  //------------------
  // unsteady: bdf1
  //------------------
  using bdf1tag = pressio::ode::implicitmethods::Euler;
  // default
  using de_bdf1_binder_t  = impl::UnsteadyLSPGProblemBinder<mytypes, bdf1tag, 0>;
  using de_bdf1_problem_t = typename de_bdf1_binder_t::lspg_problem_t;
  // hyper-reduced
  using hr_bdf1_binder_t  = impl::UnsteadyLSPGProblemBinder<mytypes, bdf1tag, 1>;
  using hr_bdf1_problem_t = typename hr_bdf1_binder_t::lspg_problem_t;
  // masked
  using ma_bdf1_binder_t  = impl::UnsteadyLSPGProblemBinder<mytypes, bdf1tag, 2>;
  using ma_bdf1_problem_t = typename ma_bdf1_binder_t::lspg_problem_t;

  //------------------
  // unsteady: bdf2
  //------------------
  using bdf2tag = pressio::ode::implicitmethods::BDF2;
  // default
  using de_bdf2_binder_t  = impl::UnsteadyLSPGProblemBinder<mytypes, bdf2tag, 0>;
  using de_bdf2_problem_t = typename de_bdf2_binder_t::lspg_problem_t;
  // hyper-reduced
  using hr_bdf2_binder_t  = impl::UnsteadyLSPGProblemBinder<mytypes, bdf2tag, 1>;
  using hr_bdf2_problem_t = typename hr_bdf2_binder_t::lspg_problem_t;
  // masked
  using ma_bdf2_binder_t  = impl::UnsteadyLSPGProblemBinder<mytypes, bdf2tag, 2>;
  using ma_bdf2_problem_t = typename ma_bdf2_binder_t::lspg_problem_t;

  // *** tuple with all problem types ***
  using problem_types = std::tuple<
    de_steady_problem_t, ma_steady_problem_t,
    de_bdf1_problem_t, de_bdf2_problem_t,
    hr_bdf1_problem_t, hr_bdf2_problem_t,
    ma_bdf1_problem_t, ma_bdf2_problem_t
    >;

  // tuple with just the steady problem types
  using steady_problem_types = std::tuple<
    de_steady_problem_t, ma_steady_problem_t
    >;

  // tuple with just the unsteady problem types
  using unsteady_problem_types = std::tuple<
    de_bdf1_problem_t, de_bdf2_problem_t,
    hr_bdf1_problem_t, hr_bdf2_problem_t,
    ma_bdf1_problem_t, ma_bdf2_problem_t
    >;

  // binding method
  static void bind(pybind11::module & lspgModule)
  {
    // *** steady problem ***
    pybind11::module lspgSteadyModule = lspgModule.def_submodule("steady");
    {
      // default
      pybind11::module m1 = lspgSteadyModule.def_submodule("default");
      de_steady_binder_t::bind(m1);

      // masked
      pybind11::module m2 = lspgSteadyModule.def_submodule("masked");
      ma_steady_binder_t::bind(m2);
    }

    // *** unsteady problem ***
    pybind11::module lspgUnsteadyModule = lspgModule.def_submodule("unsteady");
    {
      // default LSPG
      pybind11::module m1 = lspgUnsteadyModule.def_submodule("default");
      de_bdf1_binder_t::bind(m1, "Euler");
      de_bdf2_binder_t::bind(m1, "BDF2");

      // hyper-reduced LSPG
      pybind11::module m2 = lspgUnsteadyModule.def_submodule("hyperreduced");
      hr_bdf1_binder_t::bind(m2, "Euler");
      hr_bdf2_binder_t::bind(m2, "BDF2");

      // masked LSPG
      pybind11::module m3 = lspgUnsteadyModule.def_submodule("masked");
      ma_bdf1_binder_t::bind(m3, "Euler");
      ma_bdf2_binder_t::bind(m3, "BDF2");
    }
  }
};

}}//end namespace pressio4py::rom
#endif
