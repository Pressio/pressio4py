
import numpy as np
from scipy import linalg
from pressio4py import logger, solvers, ode

class MySys1:
  def createResidual(self):
    return np.zeros(5)

  def createJacobian(self):
    return np.zeros((5,5))

  def residual(self, stateIn, R):
    for i in range(5):
      R[i] = float(i)

  def jacobian(self, stateIn, J):
    count = 0.
    for i in range(5):
      for j in range(5):
        J[i,j] = float(count)
        count += 1.

class MyLinSolver1:
  def __init__(self):
    self.callCount_ = 0

  def solve(self, A,b,x):
    print("\n Python Lin solver")
    gold_A = np.array([[ 0., 1.,  2.,  3., 4.],
                       [ 5.,  6.,  7.,  8.,  9.],
                       [10., 11., 12., 13., 14.],
                       [15., 16., 17., 18., 19.],
                      [20., 21., 22., 23., 24.]])
    gold_b = np.array([0., 1., 2., 3., 4.])
    assert(np.allclose(A, gold_A))
    assert(np.allclose(b, gold_b))
    print(A)
    print(b)

def test_newton_raphson_1():
  logger.initialize(logger.logto.terminal)
  logger.setVerbosity([logger.loglevel.debug])

  state = np.ones(5)
  sys = MySys1()
  lsO = MyLinSolver1()
  nlsO = solvers.create_newton_raphson(sys, state, lsO)
  nlsO.setUpdatingCriterion(solvers.update.Standard)
  nlsO.setMaxIterations(2)
  nlsO.setStoppingCriterion(solvers.stop.AfterMaxIters)
  nlsO.solve(sys, state)
  print(state)

  logger.finalize()


class MySys2:
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

class MyLinSolver2:
  def solve(self, A,b,x):
    print("\n Python Lin solver")
    lumat, piv, info = linalg.lapack.dgetrf(A, overwrite_a=False)
    x[:], info = linalg.lapack.dgetrs(lumat, piv, b, 0, 0)

def test_newton_raphson_2():
  print("\n")
  logger.initialize(logger.logto.terminal)
  logger.setVerbosity([logger.loglevel.debug])

  state = np.array([0.001, 0.0001])
  sys = MySys2()
  lsO = MyLinSolver2()
  nlsO = solvers.create_newton_raphson(sys, state, lsO)
  #nlsO.setUpdatingCriterion(solvers.update.Standard)
  #nlsO.setMaxIterations(2)
  #nlsO.setStoppingCriterion(solvers.stop.AfterMaxIters)
  nlsO.solve(sys, state)
  gold = np.array([1., 0.])
  np.allclose(gold, state)
  print(state)

  logger.finalize()
