
import numpy as np
from scipy import linalg
from pressio4py import logger, solvers, ode

class MySys1:
  def createResidual(self):
    return np.zeros(5)

  def createJacobian(self):
    return np.zeros((5,2))

  def residual(self, stateIn, R):
    for i in range(5):
      R[i] = float(i)

  def jacobian(self, stateIn, J):
    count = 0.
    for i in range(J.shape[0]):
      for j in range(J.shape[1]):
        J[i,j] = float(count)
        count += 1.

class MyLinSolver:
  def solve(self, A,b,x):
    print(x)
    print("Python Lin solver")
    gold_A = np.array([[120., 140.], [140., 165]])
    assert(np.allclose(gold_A, A))
    gold_b = np.array([-60., -70.])
    assert(np.allclose(gold_b, b))
    x[:] = 1

def test_gn_neq_1():
  logger.initialize(logger.logto.terminal)
  logger.setVerbosity([logger.loglevel.debug])

  state = np.ones(2)
  sys = MySys1()
  lsO = MyLinSolver()
  print("lsO address = ", hex(id(lsO)))
  nlsO = solvers.create_gauss_newton(sys, state, lsO)
  nlsO.setUpdatingCriterion(solvers.update.Standard)
  nlsO.setMaxIterations(2)
  nlsO.solve(sys, state)
  print(state)
  assert(np.allclose(state, np.array([3., 3.])))

  logger.finalize()


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

def test_gn_neq_rosenbrock():
  print("\n")
  logger.initialize(logger.logto.terminal)
  logger.setVerbosity([logger.loglevel.debug])

  state = np.array([-0.05, 1.1, 1.2, 1.5])
  sys = RosenbrockSys()
  lsO = MyLinSolver2()
  nlsO = solvers.create_gauss_newton(sys, state, lsO)
  nlsO.setTolerance(1e-5)
  nlsO.solve(sys, state)
  print(state)

  gold = np.array([1.00000001567414e+00,
                   9.99999999124769e-01,
  		   9.99999996519930e-01,
                   9.99999988898883e-01])
  assert(np.allclose(gold, state))
  logger.finalize()
