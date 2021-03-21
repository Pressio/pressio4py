
import numpy as np
from scipy import linalg

from pressio4py import logger
from pressio4py import rom as rom
from pressio4py import solvers as solvers

np.set_printoptions(linewidth=140)

#----------------------------
class MyMasker:
  def __init__(self, indices):
    self.rows_ = indices
    self.sampleMeshSize_ = len(indices)

  def createApplyMaskResult(self, operand):
    if (operand.ndim == 1):
      return np.zeros(self.sampleMeshSize_)
    else:
      return np.zeros((self.sampleMeshSize_, operand.shape[1]))

  def applyMask(self, operand, time, result):
    result[:] = np.take(operand, self.rows_, axis=0)

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
    # assert(len(R) == self.N_)
    # assert(len(ynp1) == self.N_)
    # assert(len(yn) == self.N_)
    # if self.callCount_ == 1:
    #   ynp1_gold = np.array([3.,6.,9.,12.,15.,18.,21.])
    #   assert( np.allclose(ynp1, ynp1_gold, atol=1e-12) )
    #   yn_gold = np.array([3.,6.,9.,12.,15.,18.,21.])
    #   assert( np.allclose(yn, yn_gold, atol=1e-12) )

    f = np.arange(10., 110., 10.)
    R[:] = ynp1 - yn -dt*f
    print("ynp1")
    print(ynp1)
    print("yn")
    print(yn)
    print("R")
    print(R)

  def applyDiscreteTimeJacobian(self, step, time, dt, B, A, ynp1, yn):
    assert(A.shape[0] == self.N_)
    assert(A.shape[1] == 3)
    A[:,0] = 1.;
    A[:,1] = 2.;
    A[:,2] = 3.;

#----------------------------
class MyLinSolver:
  def __init__(self):
    self.callCount_ = 0

  # recall that this is called from the nonlinear solver
  # and x is the correction to apply to the nonlinear state
  def solve(self, A, b, x):
    self.callCount_ += 1
    x[:] = 1.

    print("\n")
    print(A)
    print(b)
    if self.callCount_ == 1:
      bGold = np.ones(3)*(-26.)
      assert( np.allclose(b, bGold, atol=1e-12) )
      hGold = np.array(([4.,8.,12.],
                        [4.,8.,12.],
                        [4.,8.,12.]))
      assert( np.allclose(A, hGold, atol=1e-12) )

#----------------------------
def test():
  '''
  check that masked Galerkin with discrete-time api works correctly
  '''
  logger.initialize(logger.logto.terminal, "null")
  logger.setVerbosity([logger.loglevel.info])

  N   = 10
  romSize = 3
  Nsteps   = 1
  dt = 0.1

  appObj  = MyTestApp(N)
  yRef    = np.zeros(N)
  yRom = np.zeros(romSize)

  # create a dummy phi = all 1s
  # and make phi column-major so decoder only views it
  # and does not make a copy of it
  phi = np.ones((N,romSize), order='F')
  decoder = rom.Decoder(phi)

  # pick sample mesh indices
  sampleMeshIndices = [2,5,6,9]
  # create phi on the "sample mesh"
  phiSM = np.take(phi, sampleMeshIndices, axis=0)
  # create projector (pass the phiSM)
  projector = rom.galerkin.ArbitraryProjector(phiSM)
  # create masker
  masker = MyMasker(sampleMeshIndices)

  problem = rom.galerkin.masked.ProblemDiscreteTimeTwoStates(appObj, decoder, yRom, yRef, masker, projector)

  # linear and non linear solver
  lsO = MyLinSolver()
  nlsO = solvers.createNewtonRaphson(problem, yRom, lsO)
  nlsO.setUpdatingCriterion(solvers.update.standard)
  nlsO.setMaxIterations(1)
  nlsO.setStoppingCriterion(solvers.stop.afterMaxIters)

  # solve
  rom.galerkin.advanceNSteps(problem, yRom, 0., dt, Nsteps, nlsO)
