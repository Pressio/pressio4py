
import numpy as np
from scipy import linalg

from pressio4py import logger
from pressio4py import rom as rom
from pressio4py import solvers as solvers

np.set_printoptions(linewidth=140)

#----------------------------
class MyTestApp:
  def __init__(self, Nst, Nsm, smInd):
    self.Nst_ = Nst
    self.Nsm_ = Nsm
    self.callCount_ = 0
    self.smIndices_ = smInd

  def createDiscreteTimeResidual(self):
    return np.zeros(self.Nsm_)

  def createApplyDiscreteTimeJacobianResult(self, B):
    return np.zeros((self.Nsm_, B.shape[1]))

  def discreteTimeResidual(self, step, time, dt, R, ynp1, yn):
    self.callCount_ += 1
    assert(len(R) == self.Nsm_)
    assert(len(ynp1) == self.Nst_)
    assert(len(yn) == self.Nst_)

    f = np.arange(10., 110., 10.)
    R[:] = ynp1[self.smIndices_] - yn[self.smIndices_] -dt*f[self.smIndices_]
    print("ynp1")
    print(ynp1)
    print("yn")
    print(yn)
    print("R")
    print(R)

  def applyDiscreteTimeJacobian(self, step, time, dt, B, A, ynp1, yn):
    #assert(A.shape[0] == self.Nsm_)
    #assert(A.shape[1] == 3)
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

  Nstencil = 10
  Nsmesh   = 4
  romSize = 3
  Nsteps   = 1
  dt = 0.1

  sampleMeshIndices = [2,5,6,9]
  appObj  = MyTestApp(Nstencil, Nsmesh, sampleMeshIndices)
  yRef    = np.zeros(Nstencil)
  yRom = np.zeros(romSize)

  # decoder is on stencil mesh
  phi = np.ones((Nstencil,romSize), order='F')
  decoder = rom.Decoder(phi)

  # projector is on sample mesh
  phiSM = np.take(phi, sampleMeshIndices, axis=0)
  projector = rom.galerkin.ArbitraryProjector(phiSM)

  # create problem
  problem = rom.galerkin.hyperreduced.ProblemDiscreteTimeTwoStates(appObj, decoder,
                                                                   yRom, yRef,
                                                                   projector)

  # linear and non linear solver
  lsO = MyLinSolver()
  nlsO = solvers.createNewtonRaphson(problem, yRom, lsO)
  nlsO.setUpdatingCriterion(solvers.update.standard)
  nlsO.setMaxIterations(1)
  nlsO.setStoppingCriterion(solvers.stop.afterMaxIters)

  # solve
  rom.galerkin.advanceNSteps(problem, yRom, 0., dt, Nsteps, nlsO)
