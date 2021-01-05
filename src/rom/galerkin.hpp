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

namespace pressio4py{ namespace rom{ namespace impl{

/*
  the problemid is used to choose the subcase:
  - default = 0
  - hyper-reduced = 1
  - masked = 2
*/
template <class mytypes, class ode_tag, int problemid, class projector_t = void>
struct GalerkinBinder
{
  using rom_native_state_t   = typename mytypes::rom_native_state_t;
  using fom_native_state_t   = typename mytypes::fom_native_state_t;
  using rom_state_t	     = typename mytypes::rom_state_t;
  using decoder_native_jac_t = typename mytypes::decoder_native_jac_t;
  using decoder_t	     = typename mytypes::decoder_t;

  using sys_wrapper_t =
    typename std::conditional<
    ::pressio::ode::predicates::is_explicit_stepper_tag<ode_tag>::value,
    pressio4py::rom::FomWrapperCTimeNoApplyJac<
      ::pressio4py::scalar_t, fom_native_state_t, fom_native_state_t>,
    pressio4py::rom::FomWrapperCTimeWithApplyJac<
      ::pressio4py::scalar_t, fom_native_state_t, fom_native_state_t, decoder_native_jac_t>
    >::type;

  // makser_t only used when problemid==2
  using masker_t = pressio4py::rom::MaskerWrapper<::pressio4py::scalar_t>;

  using galerkin_problem_t =
    typename std::conditional<
    problemid==0,
    pressio::rom::galerkin::impl::composeDefaultProblemContTime_t<
      ode_tag, sys_wrapper_t, decoder_t, rom_state_t>,
    typename std::conditional<
      problemid==1,
      pressio::rom::galerkin::impl::composeHyperReducedVelocityProblemContTime_t<
	ode_tag, sys_wrapper_t, decoder_t, rom_state_t, projector_t>,
      typename std::conditional<
	problemid==2,
	pressio::rom::galerkin::impl::composeMaskedVelocityProblemContTime_t<
	  ode_tag, sys_wrapper_t, decoder_t, rom_state_t, masker_t, projector_t>,
	void
	>::type
      >::type
    >::type;

  //using velocity_policy_t = typename galerkin_problem_t::velocity_policy_t;
  //using galerkin_stepper_t = typename galerkin_problem_t::stepper_t;

  // constructor for default explicit galerkin
  template<typename T, int _problemid = problemid>
  static typename std::enable_if<_problemid==0>::type
  bindProblemConstructor(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pybind11::object,		//adapter object directly from Python
		decoder_t &,			//decoder
		const rom_native_state_t &,	//native python rom state
		const fom_native_state_t &>()); //native python fom reference state
  }

  // constructor for hyper-reduced velocity explicit galerkin
  template<typename T, int _problemid = problemid>
  static typename std::enable_if<_problemid==1>::type
  bindProblemConstructor(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pybind11::object,	     //adapter object directly from Python
		decoder_t &,		     //decoder
		const rom_native_state_t &,  //native python rom state
		const fom_native_state_t &,  //native python fom reference state
		const projector_t &	     //projection operator
		>());
  }

  // constructor for masked velocity explicit galerkin
  template<typename T, int _problemid = problemid>
  static typename std::enable_if<_problemid==2>::type
  bindProblemConstructor(pybind11::class_<T> & problem)
  {
    problem.def(pybind11::init<
		pybind11::object,	     //adapter object directly from Python
		decoder_t &,		     //decoder
		const rom_native_state_t &,  //native python rom state
		const fom_native_state_t &,  //native python fom reference state
		pybind11::object,	     //the masker object directly from python
		const projector_t &	     //projection operator
		>());
  }

  static void bind(pybind11::module & m, const std::string appendToProblemName)
  {
    // // stepper
    // pybind11::class_<galerkin_stepper_t> galStepper(m, stepperName.c_str());
    // galStepper.def(pybind11::init<
    // 		   const rom_state_t &,
    // 		   const sys_wrapper_t &,
    // 		   const velocity_policy_t &>());

    const auto problemPythonName = "Problem"+appendToProblemName;

    // problem
    pybind11::class_<galerkin_problem_t> problem(m, problemPythonName.c_str());

    // bind constructor
    bindProblemConstructor(problem);

    // bind the method to extract the fom state reconstructor
    problem.def("fomStateReconstructor",
		&galerkin_problem_t::fomStateReconstructorCRef,
		pybind11::return_value_policy::reference);
    // galProblem.def("stepper",
    // 		   &galerkin_problem_t::stepperRef,
    // 		   pybind11::return_value_policy::reference);
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

  using tagE1 = pressio::ode::explicitmethods::Euler;
  using tagE2 = pressio::ode::explicitmethods::RungeKutta4;
  // default
  using de_euler_binder_t  = impl::GalerkinBinder<mytypes, tagE1, 0>;
  using de_euler_problem_t = typename de_euler_binder_t::galerkin_problem_t;
  using de_rk4_binder_t    = impl::GalerkinBinder<mytypes, tagE2, 0>;
  using de_rk4_problem_t   = typename de_rk4_binder_t::galerkin_problem_t;
  // hyper-reduced velocity
  using hrv_euler_binder_t  = impl::GalerkinBinder<mytypes, tagE1, 1, projector_t>;
  using hrv_euler_problem_t = typename hrv_euler_binder_t::galerkin_problem_t;
  using hrv_rk4_binder_t    = impl::GalerkinBinder<mytypes, tagE2, 1, projector_t>;
  using hrv_rk4_problem_t   = typename hrv_rk4_binder_t::galerkin_problem_t;
  // masked
  using ma_euler_binder_t  = impl::GalerkinBinder<mytypes, tagE1, 2, projector_t>;
  using ma_euler_problem_t = typename ma_euler_binder_t::galerkin_problem_t;
  using ma_rk4_binder_t    = impl::GalerkinBinder<mytypes, tagE2, 2, projector_t>;
  using ma_rk4_problem_t   = typename ma_rk4_binder_t::galerkin_problem_t;

  // tuple wiht all problems explicit in time
  using problem_types = std::tuple<
    de_euler_problem_t, de_rk4_problem_t,
    hrv_euler_problem_t, hrv_rk4_problem_t,
    ma_euler_problem_t, ma_rk4_problem_t
    >;

  static void bind(pybind11::module & m)
  {
    pybind11::module m1 = m.def_submodule("default");
    de_euler_binder_t::bind(m1, "ForwardEuler");
    de_rk4_binder_t::bind(m1, "RK4");

    pybind11::module m2 = m.def_submodule("hyperreduced");
    hrv_euler_binder_t::bind(m2, "ForwardEuler");
    hrv_rk4_binder_t::bind(m2, "RK4");

    pybind11::module m3 = m.def_submodule("masked");
    ma_euler_binder_t::bind(m3, "ForwardEuler");
    ma_rk4_binder_t::bind(m3, "RK4");
  }
};

template <typename mytypes>
struct GalerkinBinderImplicit
{
  using projector_t = typename mytypes::projector_t;

  using tagI1 = pressio::ode::implicitmethods::Euler;
  using tagI2 = pressio::ode::implicitmethods::BDF2;
  // default
  using de_bdf1_binder_t  = impl::GalerkinBinder<mytypes, tagI1, 0>;
  using de_bdf1_problem_t = typename de_bdf1_binder_t::galerkin_problem_t;
  using de_bdf2_binder_t  = impl::GalerkinBinder<mytypes, tagI2, 0>;
  using de_bdf2_problem_t = typename de_bdf2_binder_t::galerkin_problem_t;
  // hyper-reduced velocity
  using hrv_bdf1_binder_t  = impl::GalerkinBinder<mytypes, tagI1, 1, projector_t>;
  using hrv_bdf1_problem_t = typename hrv_bdf1_binder_t::galerkin_problem_t;
  using hrv_bdf2_binder_t  = impl::GalerkinBinder<mytypes, tagI2, 1, projector_t>;
  using hrv_bdf2_problem_t = typename hrv_bdf2_binder_t::galerkin_problem_t;
  // masked
  using ma_bdf1_binder_t  = impl::GalerkinBinder<mytypes, tagI1, 2, projector_t>;
  using ma_bdf1_problem_t = typename ma_bdf1_binder_t::galerkin_problem_t;
  using ma_bdf2_binder_t  = impl::GalerkinBinder<mytypes, tagI2, 2, projector_t>;
  using ma_bdf2_problem_t = typename ma_bdf2_binder_t::galerkin_problem_t;

  using problem_types = std::tuple<
    de_bdf1_problem_t, de_bdf2_problem_t,
    hrv_bdf1_problem_t, hrv_bdf2_problem_t,
    ma_bdf1_problem_t, ma_bdf2_problem_t
    >;

  static void bind(pybind11::module & m)
  {
    pybind11::module m1 = m.def_submodule("default");
    de_bdf1_binder_t::bind(m1, "BackwardEuler");
    de_bdf2_binder_t::bind(m1, "BDF2");

    pybind11::module m2 = m.def_submodule("hyperreduced");
    hrv_bdf1_binder_t::bind(m2, "BackwardEuler");
    hrv_bdf2_binder_t::bind(m2, "BDF2");

    pybind11::module m3 = m.def_submodule("masked");
    ma_bdf1_binder_t::bind(m3, "BackwardEuler");
    ma_bdf2_binder_t::bind(m3, "BDF2");
  }
};

}}//namespace pressio4py::rom
#endif
