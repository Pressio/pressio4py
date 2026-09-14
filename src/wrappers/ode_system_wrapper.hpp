#ifndef PRESSIO4PY_PYBINDINGS_ODE_SYSTEM_WRAPPER_HPP_
#define PRESSIO4PY_PYBINDINGS_ODE_SYSTEM_WRAPPER_HPP_

#include <optional>

namespace pressio4py{

// Adapt the legacy pressio4py Python ODE API (createVelocity/velocity)
// to the pressio-rom 0.17 ODE system concepts (createRhs/rhs).  The
// Python-facing API intentionally remains unchanged.
template<
  class ScalarType,
  class StateType,
  class VelocityType
  >
class OdeSystemExplicitWrapper
{
protected:
  pybind11::object pyObj_;

public:
  using scalar_type = ScalarType;
  using independent_variable_type = ScalarType;
  using state_type = StateType;
  using velocity_type = VelocityType;
  using rhs_type = VelocityType;

public:
  explicit OdeSystemExplicitWrapper(pybind11::object pyObj)
    : pyObj_(pyObj){}

  OdeSystemExplicitWrapper() = delete;
  ~OdeSystemExplicitWrapper() = default;
  OdeSystemExplicitWrapper(const OdeSystemExplicitWrapper&) = default;
  OdeSystemExplicitWrapper & operator=(const OdeSystemExplicitWrapper &) = default;

public:
  state_type createState() const{
    // Legacy pressio4py systems do not expose createState().  ODE state and
    // velocity have the same shape, so createVelocity() provides a correctly
    // sized work vector for the 0.17 stepper.
    return pyObj_.attr("createVelocity")();
  }

  rhs_type createRhs() const{
    return pyObj_.attr("createVelocity")();
  }

  velocity_type createVelocity() const{
    return createRhs();
  }

  void rhs(const state_type & state,
           const independent_variable_type time,
           rhs_type & value) const
  {
    pyObj_.attr("velocity")(state, time, value);
  }

  void velocity(const state_type & state,
                const scalar_type time,
                velocity_type & value) const
  {
    rhs(state, time, value);
  }
};


template<
  class ScalarType,
  class StateType,
  class VelocityType,
  class JacobianType
  >
class OdeSystemImplicitContTimeWrapper
{
protected:
  pybind11::object pyObj_;

public:
  using scalar_type = ScalarType;
  using independent_variable_type = ScalarType;
  using state_type = StateType;
  using velocity_type = VelocityType;
  using rhs_type = VelocityType;
  using jacobian_type = JacobianType;

public:
  explicit OdeSystemImplicitContTimeWrapper(pybind11::object pyObj)
    : pyObj_(pyObj){}

  OdeSystemImplicitContTimeWrapper() = delete;
  ~OdeSystemImplicitContTimeWrapper() = default;
  OdeSystemImplicitContTimeWrapper(const OdeSystemImplicitContTimeWrapper&) = default;
  OdeSystemImplicitContTimeWrapper & operator=(const OdeSystemImplicitContTimeWrapper &) = default;

public:
  state_type createState() const{
    return pyObj_.attr("createVelocity")();
  }

  rhs_type createRhs() const{
    return pyObj_.attr("createVelocity")();
  }

  velocity_type createVelocity() const{
    return createRhs();
  }

  jacobian_type createJacobian() const{
    return pyObj_.attr("createJacobian")();
  }

  void rhs(const state_type & state,
           const independent_variable_type time,
           rhs_type & value) const
  {
    pyObj_.attr("velocity")(state, time, value);
  }

  void velocity(const state_type & state,
                const scalar_type time,
                velocity_type & value) const
  {
    rhs(state, time, value);
  }

  void jacobian(const state_type & state,
                const scalar_type time,
                jacobian_type & jac) const
  {
    pyObj_.attr("jacobian")(state, time, jac);
  }

  void rhsAndJacobian(const state_type & state,
                      const independent_variable_type time,
                      rhs_type & value,
                      std::optional<jacobian_type*> jac) const
  {
    rhs(state, time, value);
    if (jac){
      pyObj_.attr("jacobian")(state, time, **jac);
    }
  }
};

} // end namespace pressio4py
#endif
