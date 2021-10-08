
import numpy as np
from scipy import linalg
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

class MyLinSolver2:
  def solve(self, A,b,x):
    lumat, piv, info = linalg.lapack.dgetrf(A, overwrite_a=False)
    x[:], info = linalg.lapack.dgetrs(lumat, piv, b, 0, 0)

class MyWeigher:
  def __call__(self, operand, result):
    result[:] = np.copy(operand)

def test_gn_neq_rosenbrock():
  print("\n")
  logger.initialize(logger.logto.terminal)
  logger.setVerbosity([logger.loglevel.debug])

  state = np.array([-0.05, 1.1, 1.2, 1.5])
  sys = RosenbrockSys()
  lsO = MyLinSolver2()
  w = MyWeigher()
  nlsO = solvers.create_weighted_gauss_newton(sys, state, lsO, w)
  nlsO.setTolerance(1e-5)
  nlsO.solve(sys, state)
  print(state)

  gold = np.array([1.00000001567414e+00,
                   9.99999999124769e-01,
  		   9.99999996519930e-01,
                   9.99999988898883e-01])
  assert(np.allclose(gold, state))
  logger.finalize()
