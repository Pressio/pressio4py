/*
//@HEADER
// ************************************************************************
//
// ode_collector_wrapper.hpp
//                         Pressio
//                         Copyright 2019
//    National Technology & Engineering Solutions of Sandia, LLC (NTESS)
//
// Pressio is licensed under BSD-3-Clause terms of use.
// ************************************************************************
//@HEADER
*/

#ifndef PRESSIO4PY_PYBINDINGS_ODE_COLLECTOR_WRAPPER_HPP_
#define PRESSIO4PY_PYBINDINGS_ODE_COLLECTOR_WRAPPER_HPP_

namespace pressio4py{

template<class Dummy>
class OdeCollectorWrapper
{
  pybind11::object pyObj_;

public:
  explicit OdeCollectorWrapper(pybind11::object pyObj)
    : pyObj_(pyObj){}

  template<class TimeType, class StateType>
  void operator()(::pressio::ode::StepCount step,
                  const TimeType & time,
                  const StateType & state) const
  {
    pyObj_(step.get(), time, state);
  }
};

} // end namespace pressio4py
#endif
