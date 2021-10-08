
# Nonlinear Solvers - General Info


@m_class{m-note m-default}

@parblock
Defined in module: `pressio4py.solvers`

Import as: &emsp; &emsp; &emsp; `from pressio4py import solvers`
@endparblock

<br/>

@m_class{m-note m-primary}

@parblock
At a high level, a nonlinear solver can be seen as a process that
repeatedly *updates* a given *state* until a certain *stopping* criterion is met.
This forms the basis of our design approach, and if you keep this in mind,
the details below will (hopefully) be very clear and intuitive.
@endparblock

<br/>

## Step by step guide

Using the pressio4py nonlinear solvers involves four main steps:

@m_class{m-block m-info}

@parblock
1. define your problem in the form of a class with a specific API
2. instantiate a nonlinear problem object
3. set/change (if needed) the convergence and updating criteria
4. invoke the `solve` operation
@endparblock


### 1. Your problem class

The problem is the object *defining your math system to solve*,
and is what you need to implement and provide to pressio4py to
compute the needed operators to operate on.
The problem must be an instance of a class meeting what
we call the "residual-jacobian" API:

```py
class Problem:
  def createResidual():
	# return a copy of the rank-1 residual
	return np.zeros(...)

  def createJacobian():
	# return a copy of the rank-2 jacobian
	return np.zeros(...)

  def residual(state, R):
    # given current state, compute residual, R

  def jacobian(state, J):
    # given current state, compute jacobian, J
```

@m_class{m-note m-warning}

@parblock
Currently, we only support dense Jacobians.
This is because pybind11 does not yet
support [passing sparse types by reference](https://pybind11.readthedocs.io/en/stable/advanced/cast/eigen.html).
Note, however, that this is not critical for the main purpose
of this library because ROMs are inherently dense.
@endparblock


<br/>

### 2. Instantiating a solver

We currently support the following methods:

| Name                | Doc                                                 | Purpose:                                                                                                                                                                                               |
|---------------------|-----------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Newton-Raphson      | [Link](./md_pages_components_nonlinsolvers_nr.html) | Systems of nonlinear equations (see e.g. [link](https://link.springer.com/content/pdf/bbm%3A978-3-319-69407-8%2F1.pdf), [link](https://www.cmu.edu/math/undergrad/suami/pdfs/2014_newton_method.pdf) ) |
| Gauss-Newton        | [Link](./md_pages_components_nonlinsolvers_gn.html) | Nonlinear least-squares problem            (see [link](https://en.wikipedia.org/wiki/Gauss%E2%80%93Newton_algorithm) )                                                                                 |
| Levenberg–Marquardt | [Link](./md_pages_components_nonlinsolvers_lm.html) | Nonlinear least-squares problem             (see [link](https://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm) )                                                                         |
<!-- | Iteratively reweighted least squares (irls) | [Link](./md_pages_components_nonlinsolvers_irls.html) | optimization problem formulated in a p-norm (see [link](https://en.wikipedia.org/wiki/Iteratively_reweighted_least_squares) )                                                                          | -->


To instantiate a solver, you can use specific factory functions as follows:

```py
solver = solvers.create_newton_raphson(problem, state, ...);
solver = solvers.create_gauss_newton(problem, state, ...);
solver = solvers.create_levenber_marquardt(problem, state, ...);
```

Note that the first and second arguments are your problem object and the state.
These are needed at construction because presssio4py uses them to initialize
all data structures needed. Please refer to each method's documentation
for the details on the other arguments to pass.


<br/>

### 3. Setting convergence and updating criteria

The solver class exposes the following methods:

```py
class Solver:

  # set stopping criterion
  def setStoppingCriterion(value) # choose value is from the 'stop' enum

  # query stopping criterion
  def stoppingCriterion():
    return # the stored stopping criterion

  # set update criterion
  def setUpdatingCriterion(value) # choose value is from the 'update' enum

  # query update criterion
  def updatingCriterion():
	return # the stored update criterion

  # set max number of iterations
  def setMaxIterations(integer)
  # query max number of iterations
  def maxIterations():
    return # the current max num of iterations

  # this is used to set a single tol for all
  def setTolerance(float)

  # finer-grained methods for setting tolerances
  def setCorrectionAbsoluteTolerance(float)
  def setCorrectionRelativeTolerance(float)
  def setResidualAbsoluteTolerance(float)
  def setResidualRelativeTolerance(float)
  def setGradientAbsoluteTolerance(float)
  def setGradientRelativeTolerance(float)

  # querying tolerances
  def correctionAbsoluteTolerance(): return #...
  def correctionRelativeTolerance(): return #...
  def residualAbsoluteTolerance()  : return #...
  def residualRelativeTolerance()  : return #...
  def gradientAbsoluteTolerance()  : return #...
  def gradientRelativeTolerance()  : return #...
};
```

The convergence criterion and associated tolerance are used to decide
why and when the solver needs to terminate.
We currently support these termination criteria:

| Enum value                                       | Description      | Currently supported for: |
|--------------------------------------------------|------------------|--------------------------|
| `stop.AfterMaxIters`                            | self-explanatory | all algorithms           |
| `stop.WhenCorrectionAbsoluteNormBelowTolerance` | self-explanatory | all algorithms           |
| `stop.WhenCorrectionRelativeNormBelowTolerance` | self-explanatory | all algorithms           |
| `stop.WhenResidualAbsoluteNormBelowTolerance`   | self-explanatory | all algorithms           |
| `stop.WhenResidualRelativeNormBelowTolerance`   | self-explanatory | all algorithms           |
| `stop.WhenGradientAbsoluteNormBelowTolerance`   | self-explanatory | least-squares solvers    |
| `stop.WhenGradientRelativeNormBelowTolerance`   | self-explanatory | least-squares solvers    |


The update stage represents the *how* the current correction term is combined
with state to update the latter. We currently support the following:

| Name         | Enum value           | Description                         | Currently supported for: |
|--------------|----------------------|-------------------------------------|--------------------------|
| Default      | `update.Standard`    | @f$x_{n+1} = x_{n} + \lambda_{n}@f$ | all algorithms           |
| Armijo       | `update.Armijo`      | todo                                | Gauss-Newton             |
| LM-schedule1 | `update.LMSchedule1` | todo                                | Levenberg–Marquardt      |
| LM-schedule2 | `update.LMSchedule2` | todo                                | Levenberg–Marquardt      |

where @f$\lambda_{n}@f$ is the correction computed at the n-th iteration of the solver.


@m_class{m-note m-info}

@par By default, a nonlinear solver uses:
- update: `update.Standard`;
- stopping: `stop.WhenCorrectionAbsoluteNormBelowTolerance`;
- max number of iterations = 100
- tolerance = 0.000001 (for everything)
@endparblock


<br/>

### 4. Invoking the solve

This is best explained via a simple snippet:

```py
from pressio4py import solvers

# assuming problem is already defined

state = np.array( # whatever initial condition )
solver = solvers.create_gauss_newton(problem, state, ...)
solver.solve(problem, state)
```
