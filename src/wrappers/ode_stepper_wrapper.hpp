
#ifndef PRESSIO4PY_PYBINDINGS_ODE_STEPPER_WRAPPER_HPP_
#define PRESSIO4PY_PYBINDINGS_ODE_STEPPER_WRAPPER_HPP_

namespace pressio4py{

template<class ScalarType, class StateType>
class OdeStepperWrapper
{

protected:
  pybind11::object pyObj_;

public:
  explicit OdeStepperWrapper(pybind11::object pyObj)
    : pyObj_(pyObj){}

  OdeStepperWrapper() = delete;
  ~OdeStepperWrapper() = default;
  OdeStepperWrapper(const OdeStepperWrapper&) = default;
  OdeStepperWrapper & operator=(const OdeStepperWrapper &) = default;
  // note, we don't declare move constructors because pybind11::object
  // gives troubles so just use copy

public:
  void operator()(StateType state,
		  const ScalarType time,
		  const ScalarType dt,
		  int32_t step_count)
  {
    pyObj_.attr("__call__")(state, time, dt, step_count);
  }
};

}
#endif
