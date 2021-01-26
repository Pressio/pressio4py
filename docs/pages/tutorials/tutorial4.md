
# Tutorial: Masked Galerkin with explicit time stepping

@m_class{m-block m-info}

@par
This tutorial shows how to create and solve a time-explicit *masked* Galerkin problem.

# What is a masked Galerkin problem?

In [this previous tutorial](./md_pages_tutorials_tutorial3.html) we introduced the *default Galerkin problem*
and explained that it is inefficient for large systems because, at every time step,
one has to compute the FOM velocity and project it using the basis matrix,
both of which scale with the FOM degrees of freedom.
This implies that even if the reduced system to integrate in time is much smaller,
the computational gains are practically zero becuase of the above bottleneck.

To overcome this obstacle, one can rely on hyper-reduction techniques:
hyper-reduction aims at approximating the FOM nonlinear operators for a fraction
of the computational cost.
There are several hyper-reduction methods available (todo, cite).

A key aspect of hyper-reduction is that to get the most benefits out of it
from a computational cost viewpoint, one has to implement it directly inside the FOM code.
In practice, oversimplifying a bit, we can say that at its core, it involves enabling
the FOM code to compute the FOM velocity at only a subset of the mesh.
A few questions then arise: why should one risk investing the time in
implementing such technique without assessing upfront whether it will work?
how can one evaluate and compare different hyper-reduction techniques without modifying the FOM code?

@m_class{m-block m-success}

@par
pressio4py provides a variant of the Galerkin problem,
called *masked Galerkin*, that allows one to test and compare the *accuracy* of various
hyper-reduction techniques **without** having to change the FOM code.


@m_class{m-block m-info}

@par
This tutorial introduces the concept of the *mask* by showing how
to construct a masked Galerkin problem using the most
basic hyper-reduction technique, namely *collocation*.


# Masked Galerkin with collocation

In pressio4py, a *masked Galerkin* with collocation problem is defined as:
@f[
\dot{\hat{\mathbf{y}}}(t;\mathbf{\mu}) =
\mathbf{(A\phi)}^T
A\mathbf{f}
\Big(\mathbf{y}_{ref}(\mathbf{\mu})
+ \mathbf{\phi}\hat{\mathbf{y}} \Big)
@f]

where @f$\hat{y}@f$ is the reduced state,
@f$y@f$ is the full-order model (FOM) state,
@f$y_{ref}@f$ is a reference FOM state, @f$\phi@f$ is the orthonormal basis,
@f$f(...)@f$ is the FOM velocity and @f$A@f$ is a sampling matrix that picks
only target rows of @f$\phi@f$ and @f$f@f$.
Note that the only difference with the Galerkin formulation presented
in [this tutorial](./md_pages_tutorials_tutorial3.html) is the
presence of the sampling operator @f$A@f$.

Schematically, the system above corresponds to the figure below.
@image html tut_gal_2_f1.png width=65%

@par
To define the sampling matrix there are various techniques (todo, add reference to pressio-tools etc).
The most basic one is just random sampling: given the set of indices of the full mesh,
one simply randomly picks a subset of it.


@m_class{m-block m-success}

@par
Note that to access the masking functionality you don't need to change the FOM applicatio.
This is because the FOM always handles the full operators, and the masking is applied
*after* the FOM computes the velocity.
In other words, to access the masked Galerkin you can use the same FOM object you use
for the default Galerkin problem discussed in [this tutorial](./md_pages_tutorials_tutorial3.html).


# How to create a masked Galerkin problem?

In practice, for a *masked* Galerkin problem with collocation one needs:
<!-- 1. a FOM object satisfying the API described [here](file:///Users/fnrizzi/Desktop/work/ROM/gitrepos/pressio4py/docs/html/md_pages_prepare_your_app.html): note that this is a regular FOM object, nothing needs to change -->
<!-- 2. a linear decoder (see [this tutorial](./md_pages_tutorials_tutorial1.html)) -->
<!-- 3. a masker object: the role of the masker is to extract from an operand the rows needed -->

1. to create the decoder on the **FULL** mesh
```py
# e.g. let phi contain the POD modes on the full mesh
linearDecoder = rom.Decoder(phi)
```

2. to select the row indices to use for the collocation
```py
# as anticipated above, there are various methods to select indices,
# the most basic one is random sampling
collocationIndices = np.array([2,3,10,56, whatever])
```

3. to create a "projector operator" that is responsible to project the FOM velocity.
Basically, this projector knows how to compute the action of @f$(A\phi)^T@f$.
```py
modesOnSampleMesh = np.take(modes, collocationIndices, axis=0)
projector = rom.galerkin.ArbitraryProjector(modesOnSampleMesh)
```

3. creating a masker object: the masker is responsible to
act on the FOM velocity vector and "mask" it to return the collocated values.
```py
class MyMasker:
  def __init__(self, indices):
    self.rows_ = indices
    self.collocationSize_ = len(indices)

  def createApplyMaskResult(self, operand):
    return np.zeros(self.collocationSize_)

  def applyMask(self, operand, time, result):
    result[:] = np.take(operand, self.rows_)

masker = MyMasker(collocationIndices)
```


4. creating the actual masked Galerkin problem
```py
problem = rom.galerkin.masked.ProblemForwardEuler(fomObj, linearDecoder, romState, fomReferenceState, masker, projector)
```





<!-- To create a default Galerkin problem object, one needs: -->
<!-- 1. a FOM object satisfying the API described [here](file:///Users/fnrizzi/Desktop/work/ROM/gitrepos/pressio4py/docs/html/md_pages_prepare_your_app.html) -->
<!-- 2. a linear decoder (see [this tutorial](./md_pages_tutorials_tutorial1.html)) -->
<!-- 3. a rom state -->
<!-- 4. a FOM reference state -->

<!-- The synopsis is as follows: -->

<!-- ```py -->
<!-- problem = rom.galerkin.default.ProblemForwardEuler(fomObj, decoder, yRom, yRef) -->
<!-- ``` -->
<!-- Here we highlight that the problem class is within the `default` -->
<!-- module and that the time stepping scheme is part of the class name. -->
<!-- This stems from the fact that the Python bindings are built -->
<!-- from the C++ library, which is heavy on templates, thus leading to this solution. -->

<!-- To select a different time stepping scheme, one can change the last -->
<!-- part of the class name. -->
<!-- We currently support forward Euler and 4th-order Runge Kutta, and are -->
<!-- adding several others. The doc will be updated as we make progress. -->
<!-- For RK4, one would do: -->

<!-- ```py -->
<!-- problem = rom.galerkin.default.ProblemRK4(fomObj, decoder, yRom, yRef) -->
<!-- ``` -->

<!-- # How to solve a default Galerkin problem? -->

<!-- Once the target problem object is created, the reduced system -->
<!-- can be integrated in time. Here we provide the most basic function -->
<!-- to do so, which advances the system for a fixed number of steps. -->
<!-- Synopsis: -->

<!-- ```py -->
<!-- rom.galerkin.advanceNSteps(problem,     # problem object -->
<!-- 				           yRom,        # rom state to advance -->
<!-- 						   t0,          # initial time -->
<!-- 						   dt,          # time step -->
<!-- 						   Nsteps       # number of steps -->
<!-- 						   [, observer] # optional observer (see below) -->
<!-- 						   ) -->
<!-- ``` -->
<!-- The optional argument allows one to pass an "observer" object whose -->
<!-- purpose is to monitor the evolution of the reduced state. -->
<!-- The observer is called back by pressio4py during the time integration -->
<!-- at every time step. This can be useful to, e.g., save the -->
<!-- generalized coordinates, or usign them to perfom some other operation. -->

<!-- The observer class must meee the following API: -->
<!-- ```py -->
<!-- class OdeObserver: -->
<!--   def __init__(self): pass -->

<!--   def __call__(self, timeStep, time, romState): -->
<!-- 	# do what you want with romState -->
<!-- ``` -->
<!-- Note that we are working on enriching the API to integrate in time. -->
<!-- For example, we will soon support function class to advance the problem -->
<!-- until a condition is met, or until a target time is reached. -->


<!-- # Want to see all the above pieces in action? -->

<!-- Look at [this demo](./md_pages_demos_demo1.html) that uses -->
<!-- default Galerkin for a 1d PDE. -->


<!-- # Some considerations -->
<!-- @m_class{m-block m-warning} -->

<!-- @par -->
<!-- One might wonder how the above formulation can be efficient, -->
<!-- given that the right-hand side of the reduced system scales -->
<!-- with the FOM degrees of freedom. -->
<!-- This is true: the reduced system obtained from a -->
<!-- *default* problem reduces the spatial degrees of freedom, -->
<!-- but is typically not efficient because at every evaluation of the RHS, -->
<!-- it requires a large matrix vector product. -->
<!-- Thus, a default Galerkin is typically used for exploratory -->
<!-- analysis when computational efficiency is **not** a primary -->
<!-- goal, e.g. to test the feasibility of ROMs for a target problem, -->
<!-- or try different basis. -->
<!-- When computational efficiency is critical, one needs to -->
<!-- resort to hyper-reduction techniques to reduce the cost of the matrix-vector -->
<!-- product. This is covered in subsequent tutorials. -->
