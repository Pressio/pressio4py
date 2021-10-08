
# Adapter API

## What is it? Why and where is it needed?

@m_class{m-frame m-warning}

An adapter class allows an application to expose data
via an API conforming to Pressio requirements.
This interface enables pressio4py to communicate with your application
and vice versa.
Note that this step only needs to be done once: the same interface
class can then be used to run all the ROMs in pressio4py.

In other words, the FOM adapter class wraps/encapsulates your
application such that an instance of that class stores all
the information defining your target problem,
and exposes some functionalities needed to query some information.
<!-- To use the functionalities in pressio, obviously there needs to be
a way to exchange data/information between pressio and your FOM application.
To do so, in pressio we leverage the idea of an *adapter class* as a layer
allowing to standardize the way pressio interfaces with any application.
Schematically, the flow of interfation is shown below:
@image html schem.svg width=65%
 -->

An instance of the adapter class is needed to construct a ROM problem.
For the sake of the argument, let us say we are doing
[Galerkin ROM](https://pressio.github.io/algos/galerkin/)
with explicit Runge-Kutta4 time stepping.
A synopsis of its usage, just highlighting the important parts to
convey the message, would be:
```py
if __name__ == "__main__":
  # ...
  # create fom object
  fomObj = FomAdapter(#pass whatever you need to setup the problem)
  # ...
  # create ROM problem
  romProblem = rom.galerkin.default.ProblemRK4(fomObj, ...)
```
You can view more examples in the [demos](./md_pages_demos_demo1.html).

Below we discuss the API variants that an adapter class needs
to meet to interface with pressio4py.

<br>

@m_class{m-note m-success}

**CONTINUOUS-TIME API**



This API is intended for any system expressible in *time-continuous* form as
@f[
\frac{d \boldsymbol{y}}{dt} =
\boldsymbol{f}(\boldsymbol{y},t; \boldsymbol{\mu}),
\quad \boldsymbol{y}(0;\boldsymbol{\mu}) = \boldsymbol{y}(\boldsymbol{\mu}),
@f]
where @f$y@f$ is the full-order model (FOM) state,
@f$f@f$ the FOM velocity, and @f$t@f$ is time.

We envision two scenarios:
* (A) you are only able (or want) to expose the right-hand-side (or velocity) of your FOM application
* (B) you expose the right-hand-side of your FOM application as well as
the action of its Jacobian on some operand

### A: Exposes only the velocity
```py
class FomAdapter:
def __init__(self, *args):
  # initialize as you want/needed by your application
  # e.g. mesh, inputs, bc, commandline aguments, etc.

  # create f(y,t,...)
  def createVelocity():
	# say N is the total number of of unknowns
	return np.zeros(N)

  # compute velocity, f(y,t;...), for a given state, y, and time, t
  def velocity(self, y, t, f):
    f[:] = #compute velocity as needed
```

@m_class{m-block m-warning}

@par Where can you use the `AdapterA` version of the continuous-time API?
This version of the adapter can (currently) only be used for
doing Galerkin ROMs with explicit time stepping.


### B: Exposes velocity and the action of the Jacobian
```py
class FomAdapter:
def __init__(self, *args):
  # initialize as you want/needed by your application
  # e.g. mesh, inputs, bc, commandline aguments, etc.

  # create f(y,t,...)
  def createVelocity():
	# say N is the total number of of unknowns
	return np.zeros(N)

  # create result of df/dy*B
  # B is typically a skinny dense matrix (e.g. POD modes)
  def createApplyJacobianResult(self, B):
    return np.zeros((N, B.shape[1]))

  # compute velocity, f(y,t;...), for a given state, y
  def velocity(self, y, t, f):
    f[:] = #compute velocity as needed

  # given current state y(t):
  # compute A=df/dy*B, where B is a skinny dense matrix
  # Note that we just require the *action* of the Jacobian.
  def applyJacobian(self, y, B, t, A):
    A[:,:] = # compute A = df/dy * B as needed
```

@m_class{m-block m-warning}

@par Where can you use the `AdapterB` version of the continuous-time API?
This version of the adapter can be used in the following case:
- for Galerkin ROMs with explicit and implicit time stepping
- for LSPG
- for WLS
Note that LSPG and WLS only make sense for implicit time integration.



<br>


@m_class{m-note m-success}

**DISCRETE-TIME API**


This API is intended for any system expressible in a discrete-time form as
@f[
\boldsymbol{R}(\boldsymbol{y}, \boldsymbol{y_{n-1}}, ..., t_n, dt_n; ...) = \boldsymbol{0}
@f]
where @f$y@f$ is the full-order model (FOM) state, @f$t_n@f$ is the time at step @f$n@f$,
and @f$dt_n@f$ is the time-step size to use, and @f$R@f$ is the residual to compute.

```py
class FomAdapter:
def __init__(self, *args):
  # initialize as you want/needed by your application
  # e.g. mesh, inputs, bc, commandline aguments, etc.

  # create R(...)
  def createDiscreteTimeResidual(self):
    return np.zeros(N)

  def createApplyDiscreteTimeJacobianResult(self, B):
	return np.zeros((N, B.shape[1]))

  def discreteTimeResidual(self, step, time, dt, R, ynp1, yn, ynm1 [, ynm2]):
   R[:] = # compute discrete-time residual

  def applyDiscreteTimeJacobian(self, step, time, dt, B, A, ynp1, yn, ynm1 [, ynm2]):
   A[:,:] = # compute the action A = dR/dy_np1 B
```

@m_class{m-block m-warning}

@par Where can you use the discrete-time API?
This version of the adapter can be **only** used for:
- Galerkin with *implicit* time stepping
- LSPG
- WLS



<!-- @m_class{m-block m-info} -->

<!-- @par Should one prefer the continuous-time or discrete-time API? -->
<!-- In general, we suggest users to always prefer the continuous-time API because it is more general. -->






<!-- # Learn how to interface your app -->

<!-- @m_class{m-block m-info} -->

<!-- @par -->
<!-- This page describes how to setup the interface enabling -->
<!-- pressio4py to communicate with your application. -->
<!-- Note that this step only needs to be done once: the same interface -->
<!-- class can then be used to run all the ROMs in pressio4py. -->
<!-- By the end, it should be clear our design choice and -->
<!-- how to setup this "glue code". -->


<!-- # Is pressio4py applicable to your problem and application? -->

<!-- Pressio targets any system expressible as a parameterized -->
<!-- system of ordinary differential equations (ODEs) as -->
<!-- @f[ -->
<!-- \frac{d \boldsymbol{y}}{dt} = -->
<!-- \boldsymbol{f}(\boldsymbol{y},t; \boldsymbol{\mu}), -->
<!-- \quad \boldsymbol{y}(0;\boldsymbol{\mu}) = \boldsymbol{y}(\boldsymbol{\mu}), -->
<!-- @f] -->
<!-- where @f$\boldsymbol{y}@f$ is the state, @f$\mu@f$ are parameters, -->
<!-- @f$t@f$ is time and @f$\boldsymbol{f}(\boldsymbol{y},t; \boldsymbol{\mu})@f$ -->
<!-- is referred to as "velocity" (or RHS). -->
<!-- If your problem can be expressed as the system of ODEs above, then you -->
<!-- can use and experiment with any of the ROM algorithms implemented in pressio4py. -->
<!-- Note that this is a *practical* assessment, in the sense that -->
<!-- it only states what class of problems pressio4py targets. -->
<!-- It does not guarantee that ROMs would work well for your problem. -->
<!-- But this is why you should try using presio4py to see if ROMs can -->
<!-- be useful for you! -->


<!-- @m_class{m-block m-success} -->

<!-- @par -->
<!-- This formulation is quite general and does not make any assumption -->
<!-- on its origin: it may be derived from the spatial -->
<!-- discretization (regardless of the discretization method) -->
<!-- of a PDE problem, or from naturally discrete systems (e.g., -->
<!-- molecular-dynamics problems). -->



<!-- # What glue code do you need on your end to use pressio4py? -->

<!-- pressio4py requires your application to expose the -->
<!-- "velocity" @f$\boldsymbol{f}(\boldsymbol{y},t; \boldsymbol{\mu})@f$ -->
<!-- and (optionally) the action of the -->
<!-- Jacobian matrix @f$\partial f/\partial y@f$. -->
<!-- This design choice pivots on the generality of the formulation above. -->
<!-- We remark that the *same* expressive model/API is being used/expected -->
<!-- by other well-established Python libraries, e.g., `scipy.ode`. -->

<!-- In practice, this can be done by writing an adapter class -->
<!-- for your full-order model (FOM) that meets the -->
<!-- API required by pressio4py as follows: -->
<!-- ```py -->
<!-- class FomAdapter: -->
<!--   def __init__(self, *args): -->
<!--     # initialize as you want/needed by your application -->
<!-- 	# e.g. mesh, inputs, bc, commandline aguments, etc. -->

<!--   # compute velocity, f(y,t;...), for a given state, y -->
<!--   def velocity(self, y, t, f): -->
<!-- 	f[:] = #compute velocity as needed -->

<!--   # given current state y(t): -->
<!--   # compute A=df/dy*B, where B is a skinny dense matrix -->
<!--   # Note that we just require the *action* of the Jacobian. -->
<!--   def applyJacobian(self, y, B, t, A): -->
<!--     A[:,:] = # compute df/dy * B -->

<!--   # create f(y,t,...) -->
<!--   def createVelocity(): -->
<!-- 	return np.zeros(N) # say N is the total number of of unknowns -->

<!--   # create result of df/dy*B, B is typically a skinny dense matrix -->
<!--   def createApplyJacobianResult(self, B): -->
<!-- 	return np.zeros_like(B) -->
<!-- ``` -->

<!-- In simple words, the FomAdapter class wraps/encapsulates your -->
<!-- application, stores all the information defining your problem -->
<!-- and exposes some functionalities needed to query some information. -->

<!-- @m_class{m-block m-info} -->

<!-- @par -->
<!-- The `applyJacobian` method is needed when you do implicit time integration. -->
<!-- For explicit time stepping you only need the velocity. -->
