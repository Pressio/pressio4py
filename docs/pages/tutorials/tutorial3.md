
# Tutorial: Default Galerkin with explicit time stepping

@m_class{m-block m-info}

@par
This tutorial shows how to create and solve a *defalt* time-explicit Galerkin problem.

## What is a default Galerkin problem?

pressio4py supports different variants of Galerkin.
The "default" qualification is used to indicate a formulation that does *not* use hyper-reduction.
In practice, a *default Galerkin* problem corresponds to solving the following
problem:

@f[
\dot{\hat{\mathbf{y}}}(t;\mathbf{\mu}) =
\Big( \mathbf{A} \mathbf{\phi} \Big)^+
\mathbf{A} \mathbf{f}
\Big(\mathbf{y}_{ref}(\mathbf{\mu})
+ \mathbf{\phi}\hat{\mathbf{y}} \Big)
@f]
