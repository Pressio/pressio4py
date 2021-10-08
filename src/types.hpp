/*
//@HEADER
// ************************************************************************
//
// types.hpp
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

#ifndef PRESSIO4PY_PYBINDINGS_TYPES_HPP_
#define PRESSIO4PY_PYBINDINGS_TYPES_HPP_

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/iostream.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>

namespace pressio4py{
using scalar_t	= double;
using py_c_arr	= pybind11::array_t<scalar_t, pybind11::array::c_style>;
using py_f_arr	= pybind11::array_t<scalar_t, pybind11::array::f_style>;
}//end namespace pressio4py

#include "pressio/utils.hpp"
#include "pressio/type_traits.hpp"
#include "./rom/py_decoder.hpp"
#include "./wrappers/ode_collector_wrapper.hpp"
#include "./wrappers/fom_continuous_time_wrapper.hpp"
#include "./wrappers/fom_steady_wrapper.hpp"
#include "./wrappers/lin_solver_wrapper.hpp"
#include "./wrappers/qr_solver_wrapper.hpp"
#include "./wrappers/ode_dt_setter.hpp"
#include "./wrappers/nonlin_ls_weighting_wrapper.hpp"
#include "./wrappers/fom_discrete_time_wrapper.hpp"

namespace pressio4py{

using decoder_t =
  ::pressio4py::PyDecoder<scalar_t, py_f_arr, py_f_arr, py_c_arr>;

using rom_conttime_adapter_wrapper_type =
  ::pressio4py::rom::FomWrapperCTimeWithApplyJac<scalar_t, py_f_arr, py_f_arr, py_f_arr>;

using rom_disctime_n2_adapter_wrapper_type =
  ::pressio4py::rom::FomWrapperDiscreteTime<2, scalar_t, py_f_arr, py_f_arr, py_f_arr>;

using rom_disctime_n3_adapter_wrapper_type =
  ::pressio4py::rom::FomWrapperDiscreteTime<3, scalar_t, py_f_arr, py_f_arr, py_f_arr>;

using rom_steady_adapter_wrapper_type =
  ::pressio4py::rom::FomWrapperSteadyState<scalar_t, py_f_arr, py_f_arr, py_f_arr>;

using linear_solver_wrapper_t = ::pressio4py::LinSolverWrapper<py_f_arr>;
using qr_solver_wrapper_t = ::pressio4py::QrSolverWrapper<py_f_arr>;
using nonlin_ls_weigh_wrapper_t = ::pressio4py::NonLinLSWeightingWrapper;

using ode_observer_wrapper_type  = ::pressio4py::OdeCollectorWrapper<void>;
using ode_dt_setter_wrapper_type = ::pressio4py::OdeTimeStepSizeSetterWrapper<scalar_t>;


template<class ScalarType, class StateType, class ResidualType, class JacobianType>
class ResJacInterface{

protected:
  pybind11::object pyObj_;

public:
  using scalar_type       = ScalarType;
  using state_type	  = StateType;
  using residual_type	  = ResidualType;
  using jacobian_type	  = JacobianType;

public:
  explicit ResJacInterface(pybind11::object pyObj)
    : pyObj_(pyObj){}

  ResJacInterface() = delete;
  ~ResJacInterface() = default;
  ResJacInterface(const ResJacInterface&) = default;
  ResJacInterface & operator=(const ResJacInterface &) = default;
  // note, we don't declare move constructors because pybind11::object
  // gives troubles so just use copy

public:
  residual_type createResidual() const{
    return pyObj_.attr("createResidual")();
  }

  jacobian_type createJacobian() const{
    return pyObj_.attr("createJacobian")();
  }

  void residual(const state_type & state, residual_type & R) const{
    pyObj_.attr("residual")(state, R);
  }

  void jacobian(const state_type & state, jacobian_type & jac) const{
    pyObj_.attr("jacobian")(state, jac);
  }
};

}//end namespace pressio4py
#endif
