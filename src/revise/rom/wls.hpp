/*
//@HEADER
// ************************************************************************
//
// wls.hpp
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

#ifndef PRESSIO4PY_PYBINDINGS_WLS_HPP_
#define PRESSIO4PY_PYBINDINGS_WLS_HPP_

namespace pressio4py{ namespace rom{ namespace impl{

template<bool cont_time_api>
struct _fom_adapter;

template<>
struct _fom_adapter<true>
{
  // numstates ignored for cont-time api
  template<int numStates, typename ...Args>
  using type = pressio4py::rom::FomWrapperCTimeWithApplyJac<::pressio4py::scalar_t, Args...>;
};

template<>
struct _fom_adapter<false>
{
  template<int numStates, typename ...Args>
  using type = pressio4py::rom::FomWrapperDiscreteTime<numStates, ::pressio4py::scalar_t, Args...>;
};


template <bool cont_time_api, typename types, typename ode_tag, int problemid, int numStates=2>
struct WLSProblemBinder
{
  using rom_native_state_t   = typename types::rom_native_state_t;
  using fom_native_state_t   = typename types::fom_native_state_t;
  using rom_state_t	     = typename types::rom_state_t;
  using decoder_t	     = typename types::decoder_t;
  using decoder_native_jac_t = typename types::decoder_native_jac_t;
  using wls_hessian_t	     = typename types::lsq_hessian_t;

  using fom_adapter_t = typename _fom_adapter<cont_time_api>::template type<
    numStates, fom_native_state_t, fom_native_state_t, decoder_native_jac_t>;

  using hessian_mat_structure = pressio::matrixLowerTriangular;
  using precon_type	      = ::pressio::rom::wls::preconditioners::NoPreconditioner;
  using jacobians_update_tag  = ::pressio::rom::wls::NonFrozenJacobian;

  using wls_policy_t  = pressio::rom::wls::HessianGradientSequentialPolicy<
    fom_adapter_t, decoder_t, ode_tag,
    hessian_mat_structure, precon_type, jacobians_update_tag>;

  using wls_system_t = pressio::rom::wls::SystemHessianAndGradientApi<
    rom_state_t, decoder_t, wls_hessian_t, wls_policy_t>;

  static void bind(pybind11::module & m, const std::string appendToProblemName = "")
  {
    const auto policyPythonName = "SequentialPolicy"+appendToProblemName;
    pybind11::class_<wls_policy_t> policy(m, policyPythonName.c_str());
    policy.def(pybind11::init<
	       int,			    // rom size
	       int,			    // num steps in window
	       const decoder_t &,	    // decoder
	       pybind11::object,	    // native fom Python adapter class
	       const fom_native_state_t &   // fom state
	       >());

    const auto problemPythonName = "Problem"+appendToProblemName;
    pybind11::class_<wls_system_t> problem(m, problemPythonName.c_str());
    problem.def(pybind11::init<
		const decoder_t &,		// decoder
		const wls_policy_t &,		// policy
		const fom_native_state_t &,	// fomState init cond
		const fom_native_state_t &,	// fom nominal state
		const rom_native_state_t &	// rom initial condition
		>());

    // bind the method to extract the fom state reconstructor
    problem.def("fomStateReconstructor",
		&wls_system_t::fomStateReconstructorCRef,
		pybind11::return_value_policy::reference);
  }
};

//--------------------------
}//end namespace impl
//--------------------------

template <typename types>
struct WLSBinder
{
  using tag1 = pressio::ode::implicitmethods::Euler;
  using tag2 = pressio::ode::implicitmethods::BDF2;

  // *** CONTINUOUS-TIME API ***
  // default problem
  using de_bdf1_binder_t  = impl::WLSProblemBinder<true, types, tag1, 0>;
  using de_bdf1_system_t = typename de_bdf1_binder_t::wls_system_t;
  using de_bdf2_binder_t  = impl::WLSProblemBinder<true, types, tag2, 0>;
  using de_bdf2_system_t = typename de_bdf2_binder_t::wls_system_t;

  // *** DISCRETE-TIME API ***
  // default problem
  using de_dtapi_bdf1_binder_t  = impl::WLSProblemBinder<false, types, tag1, 0, 2>;
  using de_dtapi_bdf1_system_t = typename de_dtapi_bdf1_binder_t::wls_system_t;
  using de_dtapi_bdf2_binder_t = impl::WLSProblemBinder<false, types, tag2, 0, 3>;
  using de_dtapi_bdf2_system_t = typename de_dtapi_bdf2_binder_t::wls_system_t;

  // *** tuple with all problem types ***
  using system_types = std::tuple<
    de_bdf1_system_t,
    de_bdf2_system_t,
    de_dtapi_bdf1_system_t,
    de_dtapi_bdf2_system_t
    >;

  // binding method
  static void bind(pybind11::module & mParent)
  {
    // default
    pybind11::module m1 = mParent.def_submodule("default");
    de_bdf1_binder_t::bind(m1, "BDF1");
    de_bdf2_binder_t::bind(m1, "BDF2");
    de_dtapi_bdf1_binder_t::bind(m1, "DiscreteTimeBDF1");
    de_dtapi_bdf2_binder_t::bind(m1, "DiscreteTimeBDF2");
  }
};

}}//end namespace pressio4py::rom
#endif
