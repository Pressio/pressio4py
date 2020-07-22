/*
//@HEADER
// ************************************************************************
//
// gauss_newton.hpp
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

#ifndef PRESSIO4PY_PYBINDINGS_GAUSS_NEWTON_HPP_
#define PRESSIO4PY_PYBINDINGS_GAUSS_NEWTON_HPP_

namespace pressio4py{

template<typename steady_system_t, typename stepper_system_t, typename hessian_t, typename rom_native_state_t>
struct GaussNewtonSolverBinder
{
  static_assert(::pressio::solvers::concepts::system_residual_jacobian<stepper_system_t>::value,
  		"Currently only supporting bindings for GN for system with residual_jacobian_api");
  static_assert(::pressio::solvers::concepts::system_residual_jacobian<steady_system_t>::value,
  		"Currently only supporting bindings for GN for system with residual_jacobian_api");

  // linear solver is a pybind::object because it is passed by the user from python
  using linear_solver_t = pybind11::object;

  using nonlinear_solver_t = pressio::solvers::nonlinear::composeGaussNewton_t<
    // does not matter system_t passed here because the system_type is only used
    // to compose the solver, does not appear in the nonlinear solver classes types.
    stepper_system_t,
    pressio::solvers::nonlinear::DefaultUpdate,
    linear_solver_t, hessian_t>;

  GaussNewtonSolverBinder(pybind11::module & m)
  {
    pybind11::class_<nonlinear_solver_t>(m, "GaussNewton")
      .def(pybind11::init<const stepper_system_t &, const rom_native_state_t &, linear_solver_t &>())
      .def(pybind11::init<const steady_system_t &, const rom_native_state_t &, linear_solver_t &>())
      .def("getMaxIterations", &nonlinear_solver_t::getMaxIterations)
      .def("setMaxIterations", &nonlinear_solver_t::setMaxIterations)
      .def("getTolerance", &nonlinear_solver_t::getTolerance)
      .def("setTolerance", &nonlinear_solver_t::setTolerance)
      .def("solve", &nonlinear_solver_t::template solve<steady_system_t, rom_native_state_t>);
  }
};

}//end namespace
#endif
