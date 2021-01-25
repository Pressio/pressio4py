
# Tutorial: Default Galerkin with explicit time stepping

@m_class{m-block m-info}

@par
This tutorial shows how to create and solve a *default* time-explicit Galerkin problem.

## What is a default Galerkin problem?

pressio4py supports different variants of Galerkin, as we will show in subsequent tutorials.
The "default" qualification in pressio4py refers to a
formulation that does *not* use hyper-reduction.
Suppose that your full-order model (FOM) is written as
@f[
\frac{d \boldsymbol{y}}{dt} =
\boldsymbol{f}(\boldsymbol{y},t; \boldsymbol{\mu}),
\quad \boldsymbol{y}(0;\boldsymbol{\mu}) = \boldsymbol{y}(\boldsymbol{\mu}),
@f]

where @f$y@f$ is the FOM state and @f$f(...)@f$ is the FOM velocity.
Note that here both @f$y@f$ and @f$f@f$ are large, see figure below.
@image html tut_gal_1_f1.png width=30%

pressio4py defines a *default Galerkin* problem to be
@f[
\dot{\hat{\mathbf{y}}}(t;\mathbf{\mu}) =
\mathbf{\phi}^T
\mathbf{f}
\Big(\mathbf{y}_{ref}(\mathbf{\mu})
+ \mathbf{\phi}\hat{\mathbf{y}} \Big)
@f]

where @f$\hat{y}@f$ is the reduced state, also called generalized coordinates,
@f$y@f$ is the full-order model (FOM) state,
@f$y_{ref}@f$ is a reference FOM state, @f$\phi@f$ is the orthonormal basis, and
@f$f(...)@f$ is the FOM velocity.
@image html tut_gal_1_f2.png width=65%

## How to create a default Galerkin problem?

Here we focus on explicit time integration, leaving the implicit one
for a different tutorial.
To create a Galerkin problem, one needs:
1. a FOM object satisfying the API described [here](file:///Users/fnrizzi/Desktop/work/ROM/gitrepos/pressio4py/docs/html/md_pages_prepare_your_app.html);
2. a linear decoder object (see [this tutorial](./md_pages_tutorials_tutorial1.html));
3. a rom state
4. a FOM reference state

Synopsis:

```py
problem = rom.galerkin.default.ProblemForwardEuler(fomObj, decoder, yRom, yRef)
```
Here we highlight that the problem class is within the `default`
module and how the time-stepping scheme is part of the class name.
This stems from the fact that the Python bindings are built from the C++ library,
which is heavy on templates, thus leading to this solution.

To use a different time stepping scheme, one can simply change the last
part of the class name.
We currently support forward Euler and 4th-order Runge Kutta.
For RK4, one would do:

```py
problem = rom.galerkin.default.ProblemRK4(fomObj, decoder, yRom, yRef)
```

## How to solve a default Galerkin problem?

Once the problem object is created, one needs to integrate
in time the reduced system.
Synopsis:

```py
rom.galerkin.advanceNSteps(problem, yRom, t0, dt, Nsteps [, observer])
```
