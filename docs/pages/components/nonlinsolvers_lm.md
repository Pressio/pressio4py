
# Nonlinear Solvers: Levenberg-Marquardt


@m_class{m-note m-default}

@parblock
Defined in module: `pressio4py.solvers`

Import as: &emsp; &emsp; &emsp; `from pressio4py import solvers`
@endparblock

<br/>

## API, Parameters and Requirements

```py
solver = solvers.create_levenberg_marquardt(problem, state, linear_solver)		   (1)

solver = solvers.create_weighted_levenberg_marquardt(problem, state, \			   (2)
													 linear_solver, weigh_functor)
```

- `problem`:
  - instance of your problem meeting
  the [residual/jacobian API](md_pages_components_nonlinsolvers_general.html)

- `state`:
  - rank-1 `numpy.array` storing initial condition

- `linear_solver`:
  - an object that is used to solve the linear problem stemming from the normal equations
  - must meet the following API:
  ```py
  class LinearSolver:
	def solve(self, A, b, x):
	  '''
	  Here you need to solve Ax = b.
	  Remember that you need to properly overwrite x
	  '''
  ```

- `weigh_functor`:
  - applicable only to overload 2
  - callable that is called to apply weighting to operators at each nonlinear iteration
  - must meet the following API:
  ```py
  class WeighingFunctor
	def __call__(self, operand, result):
	  # apply your weighting to operand and store into result
	  # remember to properly overwrite result
  ```
