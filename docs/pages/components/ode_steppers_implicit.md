
# ode: implicit steppers


@m_class{m-note m-default}

@parblock
Defined in module: `pressio4py.ode`

Import as: &emsp; &emsp; &emsp; `from pressio4py import ode`
@endparblock


## Overview

Provides functionalities to create steppers for implicit methods.
Recall that implicit methods update the state of a system
by solving a system of equations involving both the current and next state.
An implicit stepper is an object that knows how to do one such *implicit* step.

Pressio implicit steppers are applicable to any system written in *continuous-time* form:
@f[
\frac{d \boldsymbol{y}}{dt} =
\boldsymbol{f}(\boldsymbol{y},t; ...)
@f]

and/or in a *discrete-time* form
@f[
\boldsymbol{R}(\boldsymbol{y}, \boldsymbol{y_{n-1}}, ..., t_n, dt_n; ...) = \boldsymbol{0}
@f]

Here, @f$y@f$ is the state, @f$f@f$ the velocity, @f$t@f$ is time, and @f$R@f$ is the residual.


## Continuous-time API, Parameters and Requirements

@code{.py}
stepper = ode.create_implicit_stepper(scheme, state, system)
@endcode

- `scheme`: enum value to set the desired stepping scheme.<br/>
  Currently, the choices are:

  | enum value    | Method                  | Discrete Residual Formula                                                                          |
  |---------------|-------------------------|----------------------------------------------------------------------------------------------------|
  | BDF1          | Backward Diff 1st order | @f$R = y_{n+1}-y_{n}- hf(t_{n+1},y_{n+1})@f$                                                       |
  | BDF2          | Backward Diff 2nd order | @f$R = y_{n+1}-{\tfrac {4}{3}}y_{n}+{\tfrac {1}{3}}y_{n-1} - {\tfrac {2}{3}}hf(t_{n+1},y_{n+1})@f$ |
  | CrankNicolson | Crank-Nicolson          | @f$R = y_{n+1}- y_{n} - {\tfrac {1}{2}} h \left( f(t_{n+1},y_{n+1}) + f(t_{n},y_{n}) \right)@f$    |

- `state`: `numpy.array` storing your state

- `system`: object defining how to create an instance of the velocity @f$f@f$ and how to compute it.<br/>
  Must conform to the following API:
  ```py
  class MySys:
	def __init__(self):
	  pass

	def createVelocity(self):
	  return np.zeros(...)

	def velocity(self, stateIn, time, f):
	  # compute f as needed
	  # f[:] = ...

	def createJacobian(self):
	  return np.zeros((...))

	def jacobian(self, stateIn, time, J):
		# compute J as needed
		# make sure to use J[:] to overwrite value
  ```

@m_class{m-note m-warning}

@parblock
Note that currently, the implicit steppers are implemented only
for dense Jacobians. This is on purpose, because pybind11 does
not support [passing by reference sparse types](https://pybind11.readthedocs.io/en/stable/advanced/cast/eigen.html).
Therefore, for the time being, we do not provide bindings
for doing implicit stepping for systems with sparse Jacobians.
Note, also, that this is not critical for the main purposes
of this library because ROMs are inherently dense.
@endparblock


### Stepper object

The returned stepper object exposes the following methods:

```py
class Stepper:

  def order():
    return # order of the step scheme of this stepper instantiation

  def __call__(state, current_time, dt, step_number, solver)

  def createResidual()
	return # a residual instance

  def createJacobian()
	return # a Jacobian instance

  def residual(state, R)
  def jacobian(state, J)
```



### What to do after a stepper is created?

Any stepper created using the functions above is guaranteed to satisfy
the "steppable" concept discussed [here](/Users/fnrizzi/Desktop/work/ROM/gitrepos/pressio/docs/html/md_pages_components_ode_advance.html).
Therefore, once you create a stepper, you can use
the [advancers](md_pages_components_ode_advance.html) to step forward or you can use your own.<br/>
An example is below:

@code{.py}
todo
@endcode
