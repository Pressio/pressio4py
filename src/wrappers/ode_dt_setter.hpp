/*
//@HEADER
// ************************************************************************
//
// ode_dt_setter.hpp
//                         Pressio
//                         Copyright 2019
//    National Technology & Engineering Solutions of Sandia, LLC (NTESS)
//
// Pressio is licensed under BSD-3-Clause terms of use.
// ************************************************************************
//@HEADER
*/

#ifndef PRESSIO4PY_PYBINDINGS_ODE_DT_SETTER_WRAPPER_HPP_
#define PRESSIO4PY_PYBINDINGS_ODE_DT_SETTER_WRAPPER_HPP_

namespace pressio4py{

template<class ScalarType>
class OdeTimeStepSizeSetterWrapper
{
  pybind11::object pyObj_;

public:
  explicit OdeTimeStepSizeSetterWrapper(pybind11::object pyObj)
    : pyObj_(pyObj){}

  void operator()(::pressio::ode::StepCount step,
                  ::pressio::ode::StepStartAt<ScalarType> time,
                  ::pressio::ode::StepSize<ScalarType> & dt) const
  {
    dt = pyObj_(step.get(), time.get()).template cast<ScalarType>();
  }
};

} // end namespace pressio4py
#endif
