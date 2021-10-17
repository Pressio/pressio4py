
# rom: Galerkin: General Info

@m_class{m-note m-info}

@parblock
This page explains the API for using the pressio Galerkin ROMs.
After reading this, you should understand what a "pressio Galerkin problem" is,
the variants we currently support, and how to use the problem after instantiating it.

If anything is unclear, and/or you have suggestions on how
to improve this page, [open an issue on github](https://github.com/Pressio/pressio4py/issues).
@endparblock

<br/>

## Everything starts with creating a problem!

The main entry point to use the pressio Galerkin ROMs is the problem class.
You create an instance of one of the supported "Galerkin problems" as:

@m_class{m-block m-primary}

@par
```py
problem = pressio4py.rom.galerkin.<keyword>ExplicitProblem(scheme, ...)
# or
problem = pressio4py.rom.galerkin.<keyword>ImplicitProblem(scheme, ...)
```
@endparblock

where `<keyword>` expresses the variant you want (more below), `scheme`
is a value from the `ode.stepscheme` enum to set the desired stepping scheme,
and the other arguments depend on the variant you choose.
If you pass an invalid scheme, you get a runtime error.

We currently offer the following variants:

@m_div{m-button m-success}
<a href="md_pages_components_rom_galerkin_default.html">
@m_div{m-medium}&ensp;&emsp;Default Problem&emsp; &ensp; @m_enddiv
@m_div{m-small} click to learn more @m_enddiv
</a> @m_enddiv

@m_div{m-button m-primary}
<a href="md_pages_components_rom_galerkin_hypred.html">
@m_div{m-medium}Hyper-reduced Problem @m_enddiv
@m_div{m-small} click to learn more @m_enddiv
</a> @m_enddiv

@m_div{m-button m-warning}
<a href="md_pages_components_rom_galerkin_masked.html">
@m_div{m-medium}&ensp;&emsp; Masked Problem&ensp;&emsp; @m_enddiv
@m_div{m-small} click to learn more @m_enddiv
</a> @m_enddiv


<br/>


## Explicit Problem API

An explicit Galerkin problem exposes the following API:

```py
class GalerkinProblem

  def __call__(state, time, time_step_size, step_count):

  def fomStateReconstructor():
};
```

@m_class{m-block m-success}

@par Main thing to remember:
An explicit Galerkin problem satisfies the [steppable concept](md_pages_components_ode_advance.html)
(specifically, behaves like an [explicit stepper](md_pages_components_ode_steppers_explicit.html)).
@endparblock

### How do I solve an EXPLICIT problem?

The following snippets illustrate some things you can do.

#### Snippet 1:

```py
scheme    = ode.stepscheme.RungeKutta4
problem   = galerkin.DefaultExplicitProblem(scheme, ...)

time, dt = 0., 0.5
for step in range(10):
  problem(romState, currTime, dt, step)
  time += dt
```

#### Snippet 2:

```py
class MyObserver:
  def __call__(self, step, time, state):
    # this is called at every step allowing you to
	# monitor and/or use the Galerkin state
    print(state)

scheme    = ode.stepscheme.RungeKutta4
problem   = galerkin.DefaultExplicitProblem(scheme, ...)
time0, dt, nSteps = 0, 0.5, 2
obs = MyObserver()
ode.advance_n_steps_and_observe(problem, romState, time0, dt, nSteps, obs)
```

<br/>
___
<br/>


## Implicit Problem API

If you create an implicit Galerkin problem, the problem exposes the following API:

```py
class GalerkinProblem

  def fomStateReconstructor():

  def __call__(state, time, time_step_size, step_count, solver):

  def createResidual()
	return # a residual instance

  def createJacobian()
	return # a Jacobian instance

  def residual(state, R)
	# evaluates the residual for the given state

  def jacobian(state, J)
	# evaluates the Jacobian for the given state

};
```

@m_class{m-block m-success}

@par Main thing to remember:
An implicit Galerkin problem satisfies the [steppable concept](md_pages_components_ode_advance.html)
(specifically, behaves like an [implicit stepper](md_pages_components_ode_steppers_implicit.html)).
@endparblock


### How do I solve an IMPLICIT problem?

Recall that doing implicit time stepping it is not as simple as explicit.
[For implicit, in fact, you also need a *solver* to compute the solution at the next step](md_pages_components_ode_steppers_implicit.html).
In the case of Galerkin, you can use a Newton-Raphson solver,
because at eaach step, you are solving a (reduced) system
of equations with as many equations as the number of unknowns.
More specifically, the system you need to solve has as many equations as the
dimensionality of your approximating subspace.
See some sample snippets below:


#### Snippet 1:

```py
class MyLinSolver:
  def solve(self, A,b,x):
    # solve Ax = b using your favority solver, like scipy

class MyObserver:
  def __call__(self, step, time, state):
    print(state)

if __name__ == "__main__":
  # ...
  # assuming romState and other things are already created

  scheme    = ode.stepscheme.BDF1
  problem   = galerkin.DefaultExplicitProblem(scheme, ...)

  lsO  = MyLinSolver()
  nlsO = solvers.create_newton_raphson(problem, romState, lsO)
  nlsO.setUpdatingCriterion(solvers.update.Standard)
  nlsO.setMaxIterations(5)
  nlsO.setStoppingCriterion(solvers.stop.AfterMaxIters)

  # use the call operator directly
  time, dt = 0., 0.5
  for step in range(10):
	problem(romState, currTime, dt, step, nlsO)
	time += dt

  # or use our own advance functions
  obs = MyObserver()
  t0, dt, nSteps = 0., 0.5, 5
  ode.advance_n_steps_and_observe(problem, t0, dt, Steps, obs, nlsO)
```

#### Snippet 2:

Here we show the scenario where you want to use your own nonlinear solver.

```py
class MyNonLinSolver:
  def __init__(self, system):
    self.R = system.createResidual()
    self.J = system.createJacobian()

  def solve(self, system, x):
	# here you have the solve problem
	# you can compute the operators as follows:
    system.residual(x, self.R);
    system.jacobian(x, self.J);


class MyObserver:
  def __call__(self, step, time, state):
    print(state)

if __name__ == "__main__":
  # ...
  # assuming romState and other things are already created

  scheme    = ode.stepscheme.BDF1
  problem   = galerkin.DefaultExplicitProblem(scheme, ...)

  customNonLinSolver = MyNonLinSolver(problem)

  # use the call operator directly
  time, dt = 0., 0.5
  for step in range(10):
	problem(romState, currTime, dt, step, customNonLinSolver)
	time += dt

  # or use our own advance functions
  obs = MyObserver()
  t0, dt, nSteps = 0., 0.5, 5
  ode.advance_n_steps_and_observe(problem, t0, dt, Steps, obs, customNonLinSolver)
```

\todo finish
