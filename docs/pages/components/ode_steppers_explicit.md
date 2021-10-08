
# ode: explicit steppers


@m_class{m-note m-default}

@parblock
Defined in module: `pressio4py.ode`

Import as: &emsp; &emsp; &emsp; `from pressio4py import ode`
@endparblock


## Overview

Applicable to systems of the form:
@f[
\frac{d \boldsymbol{y}}{dt} =
\boldsymbol{f}(\boldsymbol{y},t; ...)
@f]

where @f$y@f$ is the state, @f$f@f$ is the RHS (also called velocity below), @f$t@f$ is time.<br/>
Explicit methods calculate the state of a system at a later time
from the state of the system at the current time and potentially previous times.
In pressio, a "stepper" is an abstraction that represents the "how" to take a step.


## API, Parameters and Requirements

```cpp
stepper = ode.create_explicit_stepper(scheme, state, system)
```

- `scheme`:
  - value of the `ode.stepscheme` enum to set the desired stepping scheme.<br/>
  Current choices: `ForwardEuler`, `RungeKutta4`, `AdamsBashforth2`, `SSPRungeKutta3`.

- `state`:
  - `numpy.array` storing your state

- `system`:
  - object defining how to create an instance of the velocity @f$f@f$ and how to compute it.<br/>
  - Must expose at least the following methods:
  ```py
  class MySys:
	def createVelocity(self):
	  return np.zeros(...)

	def velocity(self, stateIn, time, f):
	  # compute f as needed
	  # f[:] = ...
  ```

## Stepper class API

Calling the factory function above returns a stepper object.
A stepper class exposes the follwing methods:

```py
class Stepper:

  def order():
    return # order of the step scheme of this stepper instantiation

  def __call__(state, current_time, dt, step_number)
```

When invoked, the call operator triggers the stepper to execute one step.
Having access to the call operator, you can perform you own advancement in time.
Alternatively, note that the `stepper` object satisfies the "steppable"
concept discussed [here](md_pages_components_ode_advance.html),
so you can pass it to the ["advance" functions](md_pages_components_ode_advance.html)
to step in time, see below for an example.

<br/>
___
<br/>

## Example usage 1

```py
import numpy as np
from pressio4py import ode

class MyOdeSystem:
  def createVelocity(self):
    return np.zeros(5)

  def velocity(self, stateIn, time, R):
    R[:] = 3.0

state   = np.ones(5)
system  = MyOdeSystem()
scheme  = ode.stepscheme.ForwardEuler
stepper = ode.create_explicit_stepper(scheme, state, system)
print("before doing one step: ", state)

# set time, step size, and step number
t, dt, step_number=0., 2., 1

# invoking the call operator makes the stepper take one step.
# You can use it to do your own stepping if needed.
stepper(state, t, dt, step_number)

# after the step, the state changes
print("after doing one step: ", state)
```

## Example usage 2

```py
import numpy as np
from pressio4py import ode

class MyOdeSystem:
  def createVelocity(self):
    return np.zeros(5)

  def velocity(self, stateIn, time, R):
    R[:] = 3.0

state   = np.ones(5)
system  = MyOdeSystem()
scheme  = ode.stepscheme.ForwardEuler
stepper = ode.create_explicit_stepper(scheme, state, system)

t0, dt, num_steps = 0., 1.2, 5
# here we use our own advance functions
ode.advance_n_steps(stepper, state, t0, dt, num_steps)
```
