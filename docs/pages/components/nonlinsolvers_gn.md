
# Nonlinear Solvers: Gauss-Newton


@m_class{m-note m-default}

@parblock
Defined in module: `pressio4py.solvers`

Import as: &emsp; &emsp; &emsp; `from pressio4py import solvers`
@endparblock

<br/>

## Gauss-Newton via Normal-Equations with optional weighting

### API, Parameters and Requirements

```py
solver = solvers.create_gauss_newton(problem, state, linear_solver)		    (1)

solver = solvers.create_weighted_gauss_newton(problem, state, \			    (2)
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

### Example usage

```py
import numpy as np
from scipy import linalg as spla
from pressio4py import logger, solvers, ode

class RosenbrockSys:
  def createResidual(self):
    return np.zeros(6)

  def createJacobian(self):
    return np.zeros((6,4))

  def residual(self, x, R):
    x1,x2,x3,x4 = x[0],x[1],x[2],x[3]
    R[0] = 10.*(x4 - x3*x3)
    R[1] = 10.*(x3 - x2*x2)
    R[2] = 10.*(x2 - x1*x1)
    R[3] = (1.-x1)
    R[4] = (1.-x2)
    R[5] = (1.-x3)

  def jacobian(self, x, J):
    x1,x2,x3 = x[0],x[1],x[2]
    J[0,2] = -20.*x3
    J[0,3] = 10.
    J[1,1] = -20.*x2
    J[1,2] = 10.
    J[2,0] = -20.*x1
    J[2,1] = 10.
    J[3,0] = -1.
    J[4,1] = -1.
    J[5,2] = -1.

class MyLinSolver:
  def solve(self, A,b,x):
    lumat, piv, info = linalg.lapack.dgetrf(A, overwrite_a=False)
    x[:], info = spla.lapack.dgetrs(lumat, piv, b, 0, 0)

if __name__ == '__main__':
  logger.initialize(logger.logto.terminal)
  logger.setVerbosity([logger.loglevel.info])

  state   = np.array([-0.05, 1.1, 1.2, 1.5])
  problem = RosenbrockSys()
  lin_s   = MyLinSolver()
  solver  = solvers.create_gauss_newton(problem, state, lin_s)
  solver.setTolerance(1e-5)
  solver.solve(problem, state)
  print(state)

  gold = np.array([1.00000001567414e+00,
				   9.99999999124769e-01,
				   9.99999996519930e-01,
				   9.99999988898883e-01])
  assert(np.allclose(gold, state))

  logger.finalize()
```


<br/>
___
<br/>


## Gauss-Newton via QR factorization

### API, Parameters and Requirements

```py
solver = solvers.create_gauss_newton_qr(problem, state, qr_solver);
```

- `problem`:
  - instance of your problem meeting
  the [residual/jacobian API](md_pages_components_nonlinsolvers_general.html)

- `state`:
  - rank-1 `numpy.array` storing initial condition

- `qr_solver`:
  - an object used for doing QR factorization and related operations
  - must meet the following API:
  ```py
  class QRSolver:
	def computeThin(self, A):
	  self.Q, self.R = np.linalg.qr(A, mode='reduced')

	def applyQTranspose(self, operand, result):
	  result[:] = self.Q.T.dot(operand)

	def applyRTranspose(self, operand, result):
	  result[:] = self.R.T.dot(operand)

	def solveRxb(self, b, x):
	  # solve: Rx = b
	  x[:] = linalg.solve(self.R, b)
  ```


### Example usage

```py

import numpy as np
from scipy import linalg as spla
from pressio4py import logger, solvers, ode

class RosenbrockSys:
  def createResidual(self):
    return np.zeros(6)

  def createJacobian(self):
    return np.zeros((6,4))

  def residual(self, x, R):
    x1,x2,x3,x4 = x[0],x[1],x[2],x[3]
    R[0] = 10.*(x4 - x3*x3)
    R[1] = 10.*(x3 - x2*x2)
    R[2] = 10.*(x2 - x1*x1)
    R[3] = (1.-x1)
    R[4] = (1.-x2)
    R[5] = (1.-x3)

  def jacobian(self, x, J):
    x1,x2,x3 = x[0],x[1],x[2]
    J[0,2] = -20.*x3
    J[0,3] = 10.
    J[1,1] = -20.*x2
    J[1,2] = 10.
    J[2,0] = -20.*x1
    J[2,1] = 10.
    J[3,0] = -1.
    J[4,1] = -1.
    J[5,2] = -1.

class MyQRSolver:
  def computeThin(self, A):
    self.Q, self.R = np.linalg.qr(A, mode='reduced')

  def applyQTranspose(self, operand, result):
    result[:] = self.Q.T.dot(operand)

  def applyRTranspose(self, operand, result):
    result[:] = self.R.T.dot(operand)

  def solveRxb(self, b, x):
    # solve: Rx = b
    x[:] = spla.solve(self.R, b)

if __name__ == '__main__':
  logger.initialize(logger.logto.terminal)
  logger.setVerbosity([logger.loglevel.debug])

  state   = np.array([-0.05, 1.1, 1.2, 1.5])
  problem = RosenbrockSys()
  qr_s    = MyQRSolver()
  solver  = solvers.create_gauss_newton_qr(problem, state, qr_s)
  solver.setTolerance(1e-5)
  solver.solve(problem, state)
  print(state)

  gold = np.array([1.00000001567414e+00,
                   9.99999999124769e-01,
				   9.99999996519930e-01,
                   9.99999988898883e-01])
  assert(np.allclose(gold, state))
  logger.finalize()

```
