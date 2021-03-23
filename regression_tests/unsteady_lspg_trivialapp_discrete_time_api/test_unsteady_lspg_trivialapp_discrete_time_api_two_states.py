
import numpy as np
from scipy import linalg

from pressio4py import logger
from pressio4py import rom as rom
from pressio4py import solvers as solvers

np.set_printoptions(linewidth=140)

#----------------------------
class MyTestApp:
  def __init__(self, N):
    self.N_ = N
    self.callCount_ = 0

  def createDiscreteTimeResidual(self):
    return np.zeros(self.N_)

  def createApplyDiscreteTimeJacobianResult(self, B):
    return np.zeros((self.N_, B.shape[1]))

  def discreteTimeResidual(self, step, time, dt, R, ynp1, yn):
    self.callCount_ += 1
    assert(len(R) == self.N_)
    assert(len(ynp1) == self.N_)
    assert(len(yn) == self.N_)

    if self.callCount_ == 1:
      ynp1_gold = np.array([3.,6.,9.,12.,15.,18.,21.])
      assert( np.allclose(ynp1, ynp1_gold, atol=1e-12) )
      yn_gold = np.array([3.,6.,9.,12.,15.,18.,21.])
      assert( np.allclose(yn, yn_gold, atol=1e-12) )

    if self.callCount_ == 2:
      ynp1_gold = np.array([6.,12.,18.,24.,30.,36.,42.])
      assert( np.allclose(ynp1, ynp1_gold, atol=1e-12) )
      yn_gold = np.array([3.,6.,9.,12.,15.,18.,21.])
      assert( np.allclose(yn, yn_gold, atol=1e-12) )

    R[:] = 1.
    print("ynp1")
    print(ynp1)
    print("yn")
    print(yn)

  def applyDiscreteTimeJacobian(self, step, time, dt, B, A, ynp1, yn):
    A[0,:] = 0.
    A[1,:] = 2.
    A[2,:] = 1.
    A[3,:] = 3.
    A[4,:] = 2.
    A[5,:] = 4.
    A[6,:] = 5.

#----------------------------
class MyLinSolver:
  def __init__(self):
    self.callCount_ = 0

  # recall that this is called from the nonlinear solver
  # and x is the correction to apply to the nonlinear state
  def solve(self, A, b, x):
    self.callCount_ += 1
    print(A)
    print(b)
    x[:] = 1.

    bGold = -np.ones(3)*(17)
    assert( np.allclose(b, bGold, atol=1e-12) )
    hGold = np.ones((3,3))*59.
    assert( np.allclose(A, hGold, atol=1e-12) )

#----------------------------
def test():
  '''
  check that lspg with discrete-time api works correctly
  '''
  logger.initialize(logger.logto.terminal, "null")
  logger.setVerbosity([logger.loglevel.info])

  Ngrid   = 7
  romSize = 3
  dt = 2.

  appObj  = MyTestApp(Ngrid)
  yRef    = np.zeros(Ngrid)
  phi     = np.zeros((Ngrid, romSize), order='F')
  for i in range(Ngrid): phi[i,:] = float(i+1)
  print("\n")
  print(phi)

  decoder = rom.Decoder(phi)
  yRom    = np.ones(romSize)
  problem = rom.lspg.unsteady.ProblemDiscreteTimeTwoStates(appObj, decoder, yRom, yRef)

  # linear and non linear solver
  lsO = MyLinSolver()
  nlsO = solvers.createGaussNewton(problem, yRom, lsO)
  nlsO.setUpdatingCriterion(solvers.update.standard)
  nlsO.setMaxIterations(2)
  nlsO.setStoppingCriterion(solvers.stop.afterMaxIters)

  # solve
  rom.lspg.solveNSequentialMinimizations(problem, yRom, 0., dt, 1, nlsO)
  assert( np.allclose(yRom, np.array([2.,2.,2.]), 1e-12) )
