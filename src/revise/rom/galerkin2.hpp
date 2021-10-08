/*
//@HEADER
// ************************************************************************
//
// galerkin.hpp
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

#ifndef PRESSIO4PY_PYBINDINGS_GALERKIN_HPP_
#define PRESSIO4PY_PYBINDINGS_GALERKIN_HPP_

namespace pressio4py{ namespace rom{ namespace impl{

/*
  problemid is used to choose the subcase:
  - default = 0
  - hyper-reduced = 1
  - masked = 2
*/
template <class mytypes, bool is_explicit, int problemid, class projector_t = void>
struct GalerkinBinder
{
  using fom_state_t   = typename mytypes::fom_state_t;
  using rom_state_t   = typename mytypes::rom_state_t;
  using decoder_jac_t = typename mytypes::decoder_jac_t;
  using decoder_t     = typename mytypes::decoder_t;

  using sys_wrapper_t =
    typename std::conditional<
    is_explicit,
    pressio4py::rom::FomWrapperCTimeNoApplyJac<
      ::pressio4py::scalar_t, fom_state_t, fom_state_t>,
    pressio4py::rom::FomWrapperCTimeWithApplyJac<
      ::pressio4py::scalar_t, fom_state_t, fom_state_t, decoder_jac_t>
    >::type;

  // masker_t only used when problemid==2
  using masker_t = pressio4py::rom::MaskerWrapper<::pressio4py::scalar_t>;

  using galerkin_problem_t =
    typename std::conditional<
    problemid==0,
    pressio::rom::galerkin::impl::composeDefaultProblemContTime_t<
      sys_wrapper_t, decoder_t, rom_state_t>,
    typename std::conditional<
      problemid==1,
      pressio::rom::galerkin::impl::composeHyperReducedVelocityProblemContTime_t<
	sys_wrapper_t, decoder_t, rom_state_t, projector_t>,
      typename std::conditional<
	problemid==2,
	pressio::rom::galerkin::impl::composeMaskedVelocityProblemContTime_t<
	  sys_wrapper_t, decoder_t, rom_state_t, masker_t, projector_t>,
	void
	>::type
      >::type
    >::type;

  // constructor for default
  template<typename T, int _problemid = problemid>
  static typename std::enable_if<_problemid==0>::type
  bindProblemConstructor(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pybind11::object,		//native Python adapter class
		decoder_t &,			//decoder
		const rom_native_state_t &,	//native python rom state
		const fom_native_state_t &>()); //native python fom reference state
  }

  // // constructor for hyper-reduced
  // template<typename T, int _problemid = problemid>
  // static typename std::enable_if<_problemid==1>::type
  // bindProblemConstructor(pybind11::class_<T> & problem)
  // {
  //   problem.def(pybind11::init<
  // 		pybind11::object,	     //native Python adapter class
  // 		decoder_t &,		     //decoder
  // 		const rom_native_state_t &,  //native python rom state
  // 		const fom_native_state_t &,  //native python fom reference state
  // 		const projector_t &	     //projection operator
  // 		>());
  // }

  // // constructor for masked
  // template<typename T, int _problemid = problemid>
  // static typename std::enable_if<_problemid==2>::type
  // bindProblemConstructor(pybind11::class_<T> & problem)
  // {
  //   problem.def(pybind11::init<
  // 		pybind11::object,	     //native Python adapter class
  // 		decoder_t &,		     //decoder
  // 		const rom_native_state_t &,  //native python rom state
  // 		const fom_native_state_t &,  //native python fom reference state
  // 		pybind11::object,	     //the masker object directly from python
  // 		const projector_t &	     //projection operator
  // 		>());
  // }

  static void bind(pybind11::module & m, const std::string appendToProblemName)
  {
    const auto problemPythonName = "Problem"+appendToProblemName;
    pybind11::class_<galerkin_problem_t> problem(m, problemPythonName.c_str());
    bindProblemConstructor(problem);
    problem.def("fomStateReconstructor",
		&galerkin_problem_t::fomStateReconstructorCRef,
		pybind11::return_value_policy::reference);
  }
};

/*
  problemid is used to choose the subcase:
  - default = 0, hyper-reduced = 1, masked = 2
*/
template <class mytypes, int numStates, int problemid, class projector_t = void>
struct GalerkinBinderDiscreteTime
{
  using rom_native_state_t   = typename mytypes::rom_native_state_t;
  using fom_native_state_t   = typename mytypes::fom_native_state_t;
  using rom_state_t	     = typename mytypes::rom_state_t;
  using decoder_native_jac_t = typename mytypes::decoder_native_jac_t;
  using decoder_t	     = typename mytypes::decoder_t;

  using sys_wrapper_t = pressio4py::rom::FomWrapperDiscreteTime<
    numStates, ::pressio4py::scalar_t, fom_native_state_t,
    fom_native_state_t, decoder_native_jac_t>;

  using galerkin_problem_0_t =
    pressio::rom::galerkin::impl::composeDefaultProblemDiscTime_t<
    ::pressio::ode::implicitmethods::Arbitrary,
    sys_wrapper_t, decoder_t, rom_state_t, void,
    ::pressio::ode::types::StepperOrder<1>,
    ::pressio::ode::types::StepperTotalNumberOfStates<numStates>
    >;

  using galerkin_problem_1_t =
    pressio::rom::galerkin::impl::composeHyperReducedResidualProblemDiscTime_t<
    ::pressio::ode::implicitmethods::Arbitrary,
    sys_wrapper_t, decoder_t, rom_state_t, void, projector_t,
    ::pressio::ode::types::StepperOrder<1>,
    ::pressio::ode::types::StepperTotalNumberOfStates<numStates>
    >;

  // masker_t only used when problemid==2
  using masker_t = pressio4py::rom::MaskerWrapper<::pressio4py::scalar_t>;
  using galerkin_problem_2_t =
    pressio::rom::galerkin::impl::composeMaskedResidualProblemDiscTime_t<
    ::pressio::ode::implicitmethods::Arbitrary,
    sys_wrapper_t, decoder_t, rom_state_t, void, masker_t, projector_t,
    ::pressio::ode::types::StepperOrder<1>,
    ::pressio::ode::types::StepperTotalNumberOfStates<numStates>
    >;

  using galerkin_problem_t =
    typename std::conditional<
    problemid==0,
    galerkin_problem_0_t,
    typename std::conditional<
      problemid==1,
      galerkin_problem_1_t,
      typename std::conditional<
	problemid==2,
	galerkin_problem_2_t,
	void
	>::type
      >::type
    >::type;

  // constructor for default
  template<typename T, int _problemid = problemid>
  static typename std::enable_if<_problemid==0>::type
  bindProblemConstructor(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pybind11::object,		//native Python adapter class
		decoder_t &,			//decoder
		const rom_native_state_t &,	//native python rom state
		const fom_native_state_t &>()); //native python fom reference state
  }

  // constructor for hyper-reduced
  template<typename T, int _problemid = problemid>
  static typename std::enable_if<_problemid==1>::type
  bindProblemConstructor(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pybind11::object,	     //native Python adapter class
		decoder_t &,		     //decoder
		const rom_native_state_t &,  //native python rom state
		const fom_native_state_t &,  //native python fom reference state
		const projector_t &	     //projection operator
		>());
  }

  // constructor for masked
  template<typename T, int _problemid = problemid>
  static typename std::enable_if<_problemid==2>::type
  bindProblemConstructor(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pybind11::object,	     //native Python adapter class
		decoder_t &,		     //decoder
		const rom_native_state_t &,  //native python rom state
		const fom_native_state_t &,  //native python fom reference state
		pybind11::object,	     //the masker object directly from python
		const projector_t &	     //projection operator
		>());
  }

  static void bind(pybind11::module & m, const std::string appendToProblemName)
  {
    const auto problemPythonName = "Problem"+appendToProblemName;
    pybind11::class_<galerkin_problem_t> problem(m, problemPythonName.c_str());
    bindProblemConstructor(problem);
    problem.def("fomStateReconstructor",
		&galerkin_problem_t::fomStateReconstructorCRef,
		pybind11::return_value_policy::reference);
  }
};

//--------------------------
}//end namespace impl
//--------------------------

// binder for projector
template <typename mytypes>
void bindProjector(pybind11::module & m, std::string name)
{
  using decoder_native_jac_t = typename mytypes::decoder_native_jac_t;
  using projector_t	     = typename mytypes::projector_t;

  pybind11::class_<projector_t> arbProjPy(m, name.c_str());
  arbProjPy.def(pybind11::init<decoder_native_jac_t>());
}

// binder for galerkin explicit
template <typename mytypes>
struct GalerkinBinderExplicit
{
  using projector_t = typename mytypes::projector_t;

  // default
  using de_binder_t  = impl::GalerkinBinder<mytypes, true, 0>;
  using de_problem_t = typename de_euler_binder_t::galerkin_problem_t;
  // // hyper-reduced velocity
  // using hrv_binder_t  = impl::GalerkinBinder<mytypes, true, 1, projector_t>;
  // using hrv_problem_t = typename hrv_euler_binder_t::galerkin_problem_t;
  // // masked
  // using ma_binder_t  = impl::GalerkinBinder<mytypes, true, 2, projector_t>;
  // using ma_problem_t = typename ma_euler_binder_t::galerkin_problem_t;

  // tuple wiht all problems explicit in time
  using problem_types = std::tuple<de_problem_t>;
  //,  hrv_problem_t, ma_problem_t>;

  static void bind(pybind11::module & m)
  {
    //pybind11::module m1 = m.def_submodule("default");
    de_binder_t::bind(m, "Default");

    // pybind11::module m2 = m.def_submodule("hyperreduced");
    // hrv_binder_t::bind(m2);
    // pybind11::module m3 = m.def_submodule("masked");
    // ma_binder_t::bind(m3);
  }
};

// template <typename mytypes>
// struct GalerkinBinderImplicit
// {
//   using projector_t = typename mytypes::projector_t;

//   // *** CONTINUOUS-TIME API ***
//   using tagI1 = pressio::ode::implicitmethods::Euler;
//   using tagI2 = pressio::ode::implicitmethods::BDF2;
//   // default
//   using de_bdf1_binder_t  = impl::GalerkinBinder<mytypes, tagI1, 0>;
//   using de_bdf1_problem_t = typename de_bdf1_binder_t::galerkin_problem_t;
//   using de_bdf2_binder_t  = impl::GalerkinBinder<mytypes, tagI2, 0>;
//   using de_bdf2_problem_t = typename de_bdf2_binder_t::galerkin_problem_t;
//   // hyper-reduced velocity
//   using hrv_bdf1_binder_t  = impl::GalerkinBinder<mytypes, tagI1, 1, projector_t>;
//   using hrv_bdf1_problem_t = typename hrv_bdf1_binder_t::galerkin_problem_t;
//   using hrv_bdf2_binder_t  = impl::GalerkinBinder<mytypes, tagI2, 1, projector_t>;
//   using hrv_bdf2_problem_t = typename hrv_bdf2_binder_t::galerkin_problem_t;
//   // masked
//   using ma_bdf1_binder_t  = impl::GalerkinBinder<mytypes, tagI1, 2, projector_t>;
//   using ma_bdf1_problem_t = typename ma_bdf1_binder_t::galerkin_problem_t;
//   using ma_bdf2_binder_t  = impl::GalerkinBinder<mytypes, tagI2, 2, projector_t>;
//   using ma_bdf2_problem_t = typename ma_bdf2_binder_t::galerkin_problem_t;

//   // *** DISCRETE-TIME API ***
//   // default 2 states (y_n+1 and y_n)
//   using de_dtapi_2_binder_t  = impl::GalerkinBinderDiscreteTime<mytypes, 2, 0>;
//   using de_dtapi_2_problem_t = typename de_dtapi_2_binder_t::galerkin_problem_t;
//   // default 3 states (y_n+1 and y_n and y_n-1)
//   using de_dtapi_3_binder_t  = impl::GalerkinBinderDiscreteTime<mytypes, 3, 0>;
//   using de_dtapi_3_problem_t = typename de_dtapi_3_binder_t::galerkin_problem_t;
//   // hyper-reduced 2 states (y_n+1 and y_n)
//   using hr_dtapi_2_binder_t  = impl::GalerkinBinderDiscreteTime<mytypes, 2, 1, projector_t>;
//   using hr_dtapi_2_problem_t = typename hr_dtapi_2_binder_t::galerkin_problem_t;
//   // hyper-reduced 3 states (y_n+1 and y_n and y_n-1)
//   using hr_dtapi_3_binder_t  = impl::GalerkinBinderDiscreteTime<mytypes, 3, 1, projector_t>;
//   using hr_dtapi_3_problem_t = typename hr_dtapi_3_binder_t::galerkin_problem_t;
//   // masked 2 states (y_n+1 and y_n)
//   using ma_dtapi_2_binder_t  = impl::GalerkinBinderDiscreteTime<mytypes, 2, 2, projector_t>;
//   using ma_dtapi_2_problem_t = typename ma_dtapi_2_binder_t::galerkin_problem_t;
//   // masked 3 states (y_n+1 and y_n and y_n-1)
//   using ma_dtapi_3_binder_t  = impl::GalerkinBinderDiscreteTime<mytypes, 3, 2, projector_t>;
//   using ma_dtapi_3_problem_t = typename ma_dtapi_3_binder_t::galerkin_problem_t;

//   // *** collect all problem types ***
//   using problem_types = std::tuple<
//     // continuous-time api
//     de_bdf1_problem_t, de_bdf2_problem_t,
//     hrv_bdf1_problem_t, hrv_bdf2_problem_t,
//     ma_bdf1_problem_t, ma_bdf2_problem_t,
//     // discrete-time api
//     de_dtapi_2_problem_t, de_dtapi_3_problem_t,
//     hr_dtapi_2_problem_t, hr_dtapi_3_problem_t,
//     ma_dtapi_2_problem_t, ma_dtapi_3_problem_t
//     >;

//   static void bind(pybind11::module & m)
//   {
//     pybind11::module m1 = m.def_submodule("default");
//     de_bdf1_binder_t::bind(m1, "BackwardEuler");
//     de_bdf2_binder_t::bind(m1, "BDF2");
//     de_dtapi_2_binder_t::bind(m1, "DiscreteTimeTwoStates");
//     de_dtapi_3_binder_t::bind(m1, "DiscreteTimeThreeStates");

//     pybind11::module m2 = m.def_submodule("hyperreduced");
//     hrv_bdf1_binder_t::bind(m2, "BackwardEuler");
//     hrv_bdf2_binder_t::bind(m2, "BDF2");
//     hr_dtapi_2_binder_t::bind(m2, "DiscreteTimeTwoStates");
//     hr_dtapi_3_binder_t::bind(m2, "DiscreteTimeThreeStates");

//     pybind11::module m3 = m.def_submodule("masked");
//     ma_bdf1_binder_t::bind(m3, "BackwardEuler");
//     ma_bdf2_binder_t::bind(m3, "BDF2");
//     ma_dtapi_2_binder_t::bind(m3, "DiscreteTimeTwoStates");
//     ma_dtapi_3_binder_t::bind(m3, "DiscreteTimeThreeStates");

//   }
// };

}}//namespace pressio4py::rom
#endif
