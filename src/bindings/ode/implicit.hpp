/*
//@HEADER
// ************************************************************************
//
// implicit.hpp
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

#include <iostream>
#include <Eigen/Core>
#ifdef HAVE_PYBIND11
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#endif

#include "../../../CORE_ALL"
#include "../../../ODE_ALL"
#include "../../../SOLVERS_NONLINEAR"

namespace py = pybind11;

template <::rompp::ode::ImplicitEnum odeName>
struct PyClassHelper;

template <>
struct PyClassHelper<::rompp::ode::ImplicitEnum::Undefined>{
  static constexpr char const * base_ = "Undefined";
  static constexpr char const * derived_ = "Undefined";
  static constexpr int numAuxStates_ = -1;
};

template <>
struct PyClassHelper<::rompp::ode::ImplicitEnum::Euler>{
  static constexpr char const * base_ = "IEBase";
  static constexpr char const * derived_ = "ImplicitEuler";
  static constexpr int numAuxStates_ = 1;
};


template <
  ::rompp::ode::ImplicitEnum odeName,
  typename scalar_t,
  typename state_t,
  typename residual_t,
  typename jacobian_t
  >
struct BindingImplicitOde{

  static void add(pybind11::module & m){
    using app_t = py::object;
    using step_t = int;

    //----------------------------------------------------------------------
    // Bindings for stepper

    constexpr auto baseName = PyClassHelper<odeName>::base_;
    constexpr auto derivName = PyClassHelper<odeName>::derived_;
    constexpr auto nAuxStates = PyClassHelper<odeName>::numAuxStates_;

    // concrete stepper
    using stepper_t = ::rompp::ode::ImplicitStepper
      <odeName, state_t, residual_t, jacobian_t, app_t, scalar_t>;

    // base stepper
    using base_t = ::rompp::ode::ImplicitStepperBase<stepper_t, nAuxStates>;

    pybind11::class_<base_t>(m, baseName)
      .def("residual",
	   (residual_t (base_t::*)(const state_t &)const) &base_t::residual)
      .def("residual",
	   (void (base_t::*)(const state_t &, residual_t &)const) &base_t::residual)
      .def("jacobian",
	   (jacobian_t (base_t::*)(const state_t &)const) &base_t::jacobian)
      .def("jacobian",
	   (void (base_t::*)(const state_t &, jacobian_t &)const) &base_t::jacobian);

    // concrete stepper
    pybind11::class_<stepper_t, base_t>(m, derivName)
      .def(pybind11::init< const state_t &, const app_t & >());

    //----------------------------------------------------------------------
    // Bindings for Solver

    // linear solver type
    using linear_solver_t = py::object;

    // non-linear solver type
    using hessian_t = jacobian_t;
    using nonlin_solver_t = ::rompp::solvers::iterative::PyGaussNewton
      <stepper_t, state_t, residual_t, jacobian_t, hessian_t, linear_solver_t, scalar_t>;

    // base types
    using nls_base_t = ::rompp::solvers::NonLinearSolverBase<nonlin_solver_t>;
    using iter_base_t = ::rompp::solvers::IterativeBase<scalar_t>;

    pybind11::class_<nls_base_t>(m, "NonLinSolverBase")
      .def("solve", &nls_base_t::template solve<stepper_t, state_t>);

    pybind11::class_<iter_base_t>(m, "IterBase")
      .def("getMaxIterations", &iter_base_t::getMaxIterations)
      .def("setMaxIterations", &iter_base_t::setMaxIterations)
      .def("getTolerance", &iter_base_t::getTolerance)
      .def("setTolerance", &iter_base_t::setTolerance);

    pybind11::class_<nonlin_solver_t, iter_base_t, nls_base_t>(m, "GaussNewton")
      .def(pybind11::init<
      	   const stepper_t &,
      	   const state_t &,
      	   linear_solver_t &,
	   py::object &>());

    m.def("integrateNSteps",
  	  &::rompp::ode::integrateNSteps<
  	  stepper_t, state_t, scalar_t, step_t, nonlin_solver_t>,
  	  "Integrate N Steps");
  }
};

PYBIND11_MODULE(pyode_impl, m) {

  using scalar_t = double;
  using py_state = py::array_t<scalar_t, pybind11::array::c_style>;
  using app_t = py::object;

  constexpr auto ee = ::rompp::ode::ImplicitEnum::Euler;
  BindingImplicitOde<ee, scalar_t, py_state, py_state, py_state>::add(m);
}



// struct App{
//   using st_t = Eigen::VectorXd;
//   py::object obj_;

//   App(const pybind11::object & obj)
//     : obj_(obj){}

//   void residual(const st_t & y,
// 		st_t & R,
// 		double t) const{
//     std::cout << "Inside APP2 " << std::endl;

//     pybind11::object f_r = obj_.attr("residual");
//     auto R1 = pybind11::cast<st_t>( f_r(y, R, t) );
//     std::cout << y << " \n" << R1 << std::endl;
//   }
// };

// pybind11::class_<App>(m, "App")
//   .def(pybind11::init<const py::object &>() );


// class ImplA;

// struct EulerImpl{
//   std::string a_ = {};

//   EulerImpl() = delete;
//   ~EulerImpl() = default;

//   EulerImpl(std::string a) : a_{a}{}

//   void do_step(){
//     std::cout << " fromImpl " << a_ << std::endl;
//   }
// };

// template <typename T>
// struct traits;

// template <>
// struct traits<ImplA>{
//   using impl_t = EulerImpl;
// };



// template<typename Impl>
// class Interface{
// public:
//   Interface() = default;
//   ~Interface() = default;
//   void operator()() {
//     static_cast<Impl*>(this)->compute_impl();
//   }

// };


// class ImplA : public Interface<ImplA>{
//   using base_t = Interface<ImplA>;
//   friend base_t;

//   using impl_class_t	= typename traits<ImplA>::impl_t;
//   impl_class_t myImpl_;

// public:
//   ImplA() = delete;
//   ~ImplA() = default;
//   ImplA(std::string a) : myImpl_(a){}

// private:
// template<typename ... Args2>
//   void compute_impl(Args2 && ... args){
//     myImpl_.do_step( std::forward<Args2>(args)... );
//   }
// };

// PYBIND11_MODULE(main, m) {
//   using type = Interface<ImplA>;
//   pybind11::class_<type>(m, "Interface<ImplA>")
//     .def("__call__", &type::operator());

//   pybind11::class_<ImplA, type>(m, "ImplA")
//     .def(pybind11::init<std::string>());
// }










// template <typename T>
// void foo(T a){
//   std::cout << "foo_py " << a(0) << std::endl;
// }


// //void run_test(const std::function<int(int)>& f)
// void test(py::object & f)
// {
//   Eigen::MatrixXd A(2,2);
//   A.setConstant(2.2);

//   const py::object & f_r = f.attr("residual");
//   f_r(A, 0);
//   std::cout << "GG " << A(0,0) << std::endl;
//   //py::print( f_r(0) );
//   //std::cout << "result: " << r << std::endl;
// }

// PYBIND11_MODULE(main, m) {
//   m.def("test", &test, "adds two numbers");
//   m.def("foo", &foo<Eigen::MatrixXd>, "dkdk");
// }

// // // int add(int i, int j) {
// // //     return i + j;
// // // }

// // PYBIND11_MODULE(main, m) {
// //     m.doc() = "pybind11 example plugin";
// //     m.def("add", &add, "A function which adds two numbers");
// // }
