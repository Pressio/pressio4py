
# Nonlinear Solvers: Newton-Raphson

@m_class{m-note m-default}

@parblock
Defined in module: `pressio4py.solvers`

Import as: &emsp; &emsp; &emsp; `from pressio4py import solvers`
@endparblock


## API, Parameters and Requirements

```py
solver = solvers.create_newton_raphson(problem, state, linear_solver)
```

- `problem`:
  - instance of your problem meeting the [residual/jacobian API](md_pages_components_nonlinsolvers_general.html)

- `state`:
  - rank-1 `numpy.array` holding initial condition

- `linear_solver`:
  - an object that is used to solve the "inner" linear problem for each nonlinear iteration
  - must meet the following API:
  ```py
  class LinearSolver:
	def solve(self, A, b, x):
	  '''
	  Here you need to solve Ax = b.
	  Remember that you need to properly overwrite x
	  '''
  ```


## Example usage

```py
import numpy as np
from scipy import linalg
from pressio4py import logger, solvers, ode

class MyProblem:
  def createResidual(self):
    return np.zeros(2)

  def createJacobian(self):
    return np.zeros((2,2))

  def residual(self, x, R):
    R[0] =  x[0]*x[0]*x[0] + x[1] - 1.0
    R[1] = -x[0] + x[1]*x[1]*x[1] + 1.0

  def jacobian(self, x, J):
    J[0, 0] = 3.0*x[0]*x[0]
    J[0, 1] =  1.0
    J[1, 0] = -1.0
    J[1, 1] = 3.0*x[1]*x[1]

class MyLinSolver:
  def solve(self, A,b,x):
    # note that here using lapack is an overkill,
	# since we can just solve this 2x2 system analyticall.
	# but we do this for demonstration purposes
    lumat, piv, info = linalg.lapack.dgetrf(A, overwrite_a=False)
    x[:], info = linalg.lapack.dgetrs(lumat, piv, b, 0, 0)

if __name__ == '__main__':
   logger.initialize(logger.logto.terminal)
   logger.setVerbosity([logger.loglevel.info])

   state   = np.array([0.001, 0.0001])
   problem = MyProblem()
   lin_s   = MyLinSolver()
   solver  = solvers.create_newton_raphson(problem, state, lin_s)
   solver.setMaxIterations(10)
   solver.solve(problem, state)

   # the true solution is [1., 0.]
   # so state should be close to that
   print(state)

   logger.finalize()
```
