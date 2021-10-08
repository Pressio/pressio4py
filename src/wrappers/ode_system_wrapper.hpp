
#ifndef PRESSIO4PY_PYBINDINGS_ODE_SYSTEM_WRAPPER_HPP_
#define PRESSIO4PY_PYBINDINGS_ODE_SYSTEM_WRAPPER_HPP_

namespace pressio4py{

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
  using scalar_type       = ScalarType;
  using state_type	  = StateType;
  using velocity_type	  = VelocityType;

public:
  explicit OdeSystemExplicitWrapper(pybind11::object pyObj)
    : pyObj_(pyObj){}

  OdeSystemExplicitWrapper() = delete;
  ~OdeSystemExplicitWrapper() = default;
  OdeSystemExplicitWrapper(const OdeSystemExplicitWrapper&) = default;
  OdeSystemExplicitWrapper & operator=(const OdeSystemExplicitWrapper &) = default;
  // note, we don't declare move constructors because pybind11::object
  // gives troubles so just use copy

public:
  velocity_type createVelocity() const{
    return pyObj_.attr("createVelocity")();
  }

  void velocity(const state_type & state,
		const scalar_type time,
		velocity_type & velo) const
  {
    pyObj_.attr("velocity")(state, time, velo);
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
  using scalar_type       = ScalarType;
  using state_type	  = StateType;
  using velocity_type	  = VelocityType;
  using jacobian_type	  = JacobianType;

public:
  explicit OdeSystemImplicitContTimeWrapper(pybind11::object pyObj)
    : pyObj_(pyObj){}

  OdeSystemImplicitContTimeWrapper() = delete;
  ~OdeSystemImplicitContTimeWrapper() = default;
  OdeSystemImplicitContTimeWrapper(const OdeSystemImplicitContTimeWrapper&) = default;
  OdeSystemImplicitContTimeWrapper & operator=(const OdeSystemImplicitContTimeWrapper &) = default;
  // note, we don't declare move constructors because pybind11::object
  // gives troubles so just use copy

public:
  velocity_type createVelocity() const{
    return pyObj_.attr("createVelocity")();
  }

  jacobian_type createJacobian() const{
    return pyObj_.attr("createJacobian")();
  }

  void velocity(const state_type & state,
		const scalar_type time,
		velocity_type & velo) const
  {
    pyObj_.attr("velocity")(state, time, velo);
  }

  void jacobian(const state_type & state,
		const scalar_type time,
		jacobian_type & jac) const
  {
    pyObj_.attr("jacobian")(state, time, jac);
  }
};

}
#endif
