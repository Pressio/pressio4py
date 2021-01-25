
# Tutorial: Default Galerkin with explicit time stepping

@m_class{m-block m-info}

@par
This tutorial shows how to create and solve a *default* time-explicit Galerkin problem.

## What is a default Galerkin problem?

pressio4py supports different variants of Galerkin, as we will show in subsequent tutorials.
The "default" qualification refers to a formulation that does *not* use hyper-reduction.
In practice, a *default Galerkin* problem corresponds to solving the following
problem:

@f[
\dot{\hat{\mathbf{y}}}(t;\mathbf{\mu}) =
\mathbf{\phi}^+
\mathbf{f}
\Big(\mathbf{y}_{ref}(\mathbf{\mu})
+ \mathbf{\phi}\hat{\mathbf{y}} \Big)
@f]

where @f$\hat{y}@f$ is the reduced state, also called generalized coordinates,
@f$y@f$ is the full-order model (FOM) state,
@f$y_{ref}@f$ is the reference FOM state, @f$\phi@f$ is the basis, and
@f$f(.)@f$ is the FOM velocity.
