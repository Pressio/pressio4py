
import numpy as np
from scipy import linalg
from pressio4py import logger, solvers, ode

class MySys:
  def __init__(self):
    pass

  def createResidual(self):
    return np.zeros(2)

  def createJacobian(self):
    return np.zeros((2,2))

  def residual(self, x, R):
    x0, x1 = x[0], x[1]
    R[0] = x[0] - x1*(2. - x1*(5. - x1) ) - 13.
    R[1] = x0 - x1*(14. - x1*(1. + x1) ) - 29.

  def jacobian(self, x, J):
    x0, x1 = x[0], x[1]
    J[0,0] = 1.
    J[0,1] = -x1*(2.*x1 - 5.) + (5. - x1)*x1 - 2.
    J[1,0] = 1.
    J[1,1] = x1*(x1 + 1.) - (-2.*x1 - 1.)*x1 - 14.

class MyLinSolver:
  def solve(self, A,b,x):
    print("\n Python Lin solver")
    lumat, piv, info = linalg.lapack.dgetrf(A, overwrite_a=False)
    x[:], info = linalg.lapack.dgetrs(lumat, piv, b, 0, 0)

def test_gn_neq_1():
  print("\n")
  logger.initialize(logger.logto.terminal)
  logger.setVerbosity([logger.loglevel.debug])

  state = np.array([0.5, -2.])
  sys = MySys()
  lsO = MyLinSolver()
  nlsO = solvers.create_levenberg_marquardt(sys, state, lsO)
  nlsO.setUpdatingCriterion(solvers.update.LMSchedule1)
  nlsO.setMaxIterations(5)
  nlsO.solve(sys, state)
  np.set_printoptions(precision=15, suppress=False)
  print(state)
  gold = np.array([12.445994458964783, -0.812375408654858])
  assert(np.allclose(gold, state))

  logger.finalize()
