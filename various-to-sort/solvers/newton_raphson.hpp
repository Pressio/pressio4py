/*
//@HEADER
// ************************************************************************
//
// newton_raphson.hpp
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

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include "../../../CORE_ALL"
#include "../../../SOLVERS_NONLINEAR"

namespace py = pybind11;

PYBIND11_MODULE(pysolver, m) {

  using scalar_t = double;
  using py_state = py::array_t<scalar_t, pybind11::array::c_style>;
  using system_t = py::object;

  // because the numpy linear solver is passed by user
  using lin_solver_t = py::object;

  // type of concrete nonlinear solver
  using nonlin_solver_t = ::rompp::solvers::NewtonRaphson<scalar_t, lin_solver_t>;
  // base types
  using nonlin_base_t = ::rompp::solvers::NonLinearSolverBase<nonlin_solver_t>;
  using iter_base_t = ::rompp::solvers::IterativeBase<scalar_t>;

  pybind11::class_<nonlin_base_t>(m, "NonLinBaseNewtonRaph")
    .def("solve", &nonlin_base_t::solve<system_t, py_state>);

  pybind11::class_<iter_base_t>(m, "IterBaseNewtonRaph")
    .def("getMaxIterations", &iter_base_t::getMaxIterations)
    .def("setMaxIterations", &iter_base_t::setMaxIterations)
    .def("getTolerance", &iter_base_t::getTolerance)
    .def("setTolerance", &iter_base_t::setTolerance);

  pybind11::class_<nonlin_solver_t, nonlin_base_t, iter_base_t>(m, "NewtonRaphson")
    .def(pybind11::init<>())
    .def(pybind11::init<lin_solver_t &>());

  //-------------------------------------
}









// template <
//   typename sc_t,
//   typename st_t,
//   typename res_t,
//   typename jac_t
//   >
// struct AppWrap{

//   py::object pyApp_;

//   using scalar_type = sc_t;
//   using state_type = st_t;
//   using residual_type = res_t;
//   using jacobian_type = jac_t;

//   AppWrap() = delete;
//   AppWrap(py::object & pyApp) : pyApp_{pyApp}{}
//   ~AppWrap() = default;

//   void residual(const state_type& x, residual_type& res) const {
//     pyApp_.attr("residual2")(x, res);
//   }

//   residual_type residual(const state_type& x) const {
//     auto res = pyApp_.attr("residual1")(x);
//     return res.template cast<residual_type>();
//   }

//   void jacobian(const state_type& x, jacobian_type& jac) const {
//     pyApp_.attr("jacobian2")(x, jac);
//   }

//   jacobian_type jacobian(const state_type& x) const {
//     auto jac = pyApp_.attr("jacobian1")(x);
//     return jac.template cast<jacobian_type>();
//   }
// };
