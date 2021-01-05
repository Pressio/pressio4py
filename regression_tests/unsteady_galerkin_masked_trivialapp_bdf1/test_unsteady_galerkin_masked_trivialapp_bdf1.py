
import numpy as np
from scipy import linalg

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

#--------------------------------------
class MyTestApp:
  def __init__(self, N): self.N_ = N

  def createVelocity(self):
    return np.zeros(self.N_)

  def createApplyJacobianResult(self, B):
    return np.zeros((self.N_, B.shape[1]))

  def velocity(self, u, t, f):
    assert(len(f) == self.N_)
    f[:] = np.arange(10.,110., 10.)

  def applyJacobian(self, u, B, t, A):
    # this computes: A = J*B
    # where J is supposed to be the fom jacobian
    assert(A.shape[0] == self.N_)
    assert(A.shape[1] == 3)
    A[:,0] = 1.;
    A[:,1] = 2.;
    A[:,2] = 3.;

#----------------------------
class MyLinSolver:
  def __init__(self):
    self.callCount_ = 0

  def solve(self, A,b,x):
    self.callCount_ += 1
    x[:] = 1.

    assert(A.shape[0] == A.shape[1])
    assert(A.shape[0] == 3)
    assert(b.shape[0] == 3)

    print("\n")
    print(A)
    if self.callCount_ == 1:
      bGold = np.ones(3)*(-26.)
      assert( np.allclose(b, bGold, atol=1e-12) )
      hGold = np.array(([0.6, -0.8, -1.2],
                        [-0.4,  0.2, -1.2],
                        [-0.4, -0.8, -0.2]))
      assert( np.allclose(A, hGold, atol=1e-12) )

#----------------------------
def test():
  '''
  here we test a masked galerkin with bdf1
  using artificial numbers, i.e. just checking things are correct.

  we use:
  - N = 10 (mimic grid points)
  - romSize = 3
  - dt = 0.1
  - phi = all 1s
  - the fom velocity returns  f = [10 20 30 40 50 60 70 80 90 100]
  - the masker only picks elements at i=2,5,6,9
   so the masked velocity should be [30, 60, 70, 100]

  galerkin with bdf1 solves:
       x_1 - x0 = dt * f2

  where x0 = [0 0 0]
  f2 = [1 1 1  ^T  [30
        1 1 1       60
        1 1 1       70
        1 1 1]     100]

  so f2 = [260 260 260]
  so first newton iteration should have
     J x_k = R

  where at first solver iterations should have:
        R = -dt * f2
          = [-26 -26 -26]

        J = I - dt * phi^T * masked(fomJac * phi)
          = I - dt * phi^T * masked(A)

          A = [1 2 3;
               1 2 3;
               ...
               1 2 3]

         = [1 0 0         [4 8 12
            0 1 0   - dt   4 8 12
            0 0 1]         4 8 12]

        = [0.6 -0.8 -1.2
          -0.4  0.2 -1.2
          -0.4 -0.8 -0.2]
  '''
  N = 10
  romSize  = 3
  Nsteps   = 1
  dt       = 0.1

  appObj = MyTestApp(N)
  yRef = np.zeros(N)
  yRom = np.zeros(romSize)

  # create a dummy phi = all 1s
  # and make phi column-major so decoder only views it
  # and does not make a copy of it
  phi = np.ones((N,romSize), order='F')
  # decoder
  decoder = rom.Decoder(phi)

  # pick sample mesh indices
  sampleMeshIndices = [2,5,6,9]
  # create phi on the "sample mesh"
  phiSM = np.take(phi, sampleMeshIndices, axis=0)
  # create projector (pass the phiSM)
  projector = rom.galerkin.ArbitraryProjector(phiSM)
  # create masker and galerkin problem
  masker = MyMasker(sampleMeshIndices)
  galerkinProblem = rom.galerkin.masked.ProblemBackwardEuler(
    appObj, decoder, yRom, yRef, masker, projector)

  # linear and non linear solver
  lsO = MyLinSolver()
  nlsO = solvers.NewtonRaphson(galerkinProblem, yRom, lsO)
  nlsO.setUpdatingCriterion(solvers.update.standard)
  nlsO.setMaxIterations(1)
  nlsO.setStoppingCriterion(solvers.stop.afterMaxIters)

  rom.galerkin.advanceNSteps(galerkinProblem, yRom, 0., dt, Nsteps, nlsO)
