
# Learn how to interface your app

@m_class{m-block m-info}

@par
This page describes how to setup the interface enabling
pressio4py to communicate with your application.
Note that this step only needs to be done once: the same interface
class can then be used to run all the ROMs in pressio4py.
By the end, it should be clear our design choice and
how to setup this "glue code".


# Is pressio4py applicable to your problem and application?

Pressio targets any system expressible as a parameterized
system of ordinary differential equations (ODEs) as
@f[
\frac{d \boldsymbol{x}}{dt} =
\boldsymbol{f}(\boldsymbol{x},t; \boldsymbol{\mu}),
\quad \boldsymbol{x}(0;\boldsymbol{\mu}) = \boldsymbol{x}(\boldsymbol{\mu}),
@f]
where @f$\boldsymbol{x}@f$ is the state, @f$\mu@f$ are parameters,
@f$t@f$ is time and @f$\boldsymbol{f}(\boldsymbol{x},t; \boldsymbol{\mu})@f$
is referred to as "velocity" (or RHS).
If your problem can be expressed as the system of ODEs above, then you
can use and experiment with any of the ROM algorithms implemented in pressio4py.
Note that this is a *practical* assessment, in the sense that
it only states what class of problems pressio4py targets.
It does not guarantee that ROMs would work well for your problem.
But this is why you should try using presio4py to see if ROMs can
be useful for you!


@m_class{m-block m-success}

@par
This formulation is quite general and does not make any assumption
on its origin: it may be derived from the spatial
discretization (regardless of the discretization method)
of a PDE problem, or from naturally discrete systems (e.g.,
molecular-dynamics problems).



# What glue code do you need on your end to use pressio4py?

pressio4py requires your application to expose the
"velocity" @f$\boldsymbol{f}(\boldsymbol{x},t; \boldsymbol{\mu})@f$
and (optionally) the action of the
Jacobian matrix @f$\partial f/\partial x@f$.
This design choice pivots on the generality of the formulation above.
We remark that the *same* expressive model/API is being used/expected
by other well-established Python libraries, e.g., `scipy.ode`.

In practice, this can be done by writing an adapter class
for your full-order model (FOM) that meets the
API required by pressio4py as follows:
```py
class FomAdapter:
  def __init__(self, *args):
    # initialize as you want/needed by your application
	# e.g. mesh, inputs, bc, commandline aguments, etc.

  # compute velocity, f(x,t;...), for a given state, x
  def velocity(self, x, t, f):
	f[:] = #compute velocity as needed

  # given current state x(t):
  # compute A=df/dx*B, where B is a skinny dense matrix
  # Note that we just require the *action* of the Jacobian.
  def applyJacobian(self, x, B, t, A):
    A[:,:] = # compute df/dx * B

  # create f(x,t,...)
  def createVelocity():
	return np.zeros(N) # say N is the total number of of unknowns

  # create result of df/dx*B, B is typically a skinny dense matrix
  def createApplyJacobianResult(self, B):
	return np.zeros_like(B)
```

In simple words, the FomAdapter class wraps/encapsulates your
application, stores all the information defining your problem
and exposes some functionalities needed to query some information.

@m_class{m-block m-info}

@par
The `applyJacobian` method is needed when you do implicit time integration.
For explicit time stepping you only need the velocity.


# Where is the adapter used?

An instance of the adapter class is needed to construct a ROM problem.
For the sake of the argument, let us say we are doing
[Galerkin ROM](https://pressio.github.io/algos/galerkin/)
with explicit Runge-Kutta4 time stepping.
A synopsis of the code, just highlighting the important parts to
convey the message, would be:
```py
if __name__ == "__main__":
  # ...
  # create fom object
  fomObj = FomAdapter(#pass whatever you need to setupit)
  # ...
  # create problem
  romProblem = rom.galerkin.default.ProblemRK4(fomObj, ...)
```
For more examples, you can look at the [demos](./md_pages_demos_demo1.html).
