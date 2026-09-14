/*
//@HEADER
// ************************************************************************
//
// types.hpp
//                         Pressio
//                         Copyright 2019
//    National Technology & Engineering Solutions of Sandia, LLC (NTESS)
//
// Pressio is licensed under BSD-3-Clause terms of use.
// ************************************************************************
//@HEADER
*/

#ifndef PRESSIO4PY_PYBINDINGS_TYPES_HPP_
#define PRESSIO4PY_PYBINDINGS_TYPES_HPP_

#include <cstdint>
#include <optional>
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/iostream.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>

namespace pressio4py{
using scalar_t = double;
using py_c_arr = pybind11::array_t<scalar_t, pybind11::array::c_style>;
using py_f_arr = pybind11::array_t<scalar_t, pybind11::array::f_style>;
}//end namespace pressio4py

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


// Compatibility adapter from the historical pressio4py residual/Jacobian API
// to the fused nonlinear-system concept used by pressio-rom 0.17.
template<class ScalarType, class StateType, class ResidualType, class JacobianType>
class ResJacInterface{
protected:
  pybind11::object pyObj_;
  StateType statePrototype_;

public:
  using scalar_type = ScalarType;
  using state_type = StateType;
  using residual_type = ResidualType;
  using jacobian_type = JacobianType;

public:
  ResJacInterface(pybind11::object pyObj, const StateType & statePrototype)
    : pyObj_(pyObj), statePrototype_(statePrototype){}

  ResJacInterface() = delete;
  ~ResJacInterface() = default;
  ResJacInterface(const ResJacInterface&) = default;
  ResJacInterface & operator=(const ResJacInterface &) = default;

public:
  state_type createState() const{
    // Newer Pressio systems expose createState().  Legacy pressio4py user
    // systems generally do not, so use the state supplied to the Python
    // create_*_solver call as the shape/layout prototype.
    if (pybind11::hasattr(pyObj_, "createState")){
      return pyObj_.attr("createState")();
    }
    return pybind11::module_::import("numpy")
      .attr("zeros_like")(statePrototype_)
      .template cast<state_type>();
  }

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

  void residualAndJacobian(const state_type & state,
                           residual_type & R,
                           std::optional<jacobian_type*> jac) const
  {
    residual(state, R);
    if (jac){
      this->jacobian(state, **jac);
    }
  }
};

}//end namespace pressio4py
#endif
