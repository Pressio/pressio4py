
import numpy as np
from scipy import linalg

from pressio4py import rom as rom

np.set_printoptions(linewidth=140)

#----------------------------
class MyMasker:
  def __init__(self, indices):
    self.rows_ = indices
    self.sampleMeshSize_ = len(indices)

  def createApplyMaskResult(self, operand):
    return np.zeros(self.sampleMeshSize_)

  def applyMask(self, operand, time, result):
    result[:] = np.take(operand, self.rows_)

#--------------------------------------
class MyTestApp:
  def __init__(self, N): self.N_ = N

  def createVelocity(self):
    return np.zeros(self.N_)

  def velocity(self, u, t, f):
    assert(len(f) == self.N_)
    f[:] = np.arange(10.,110., 10.)

#----------------------------------------
class OdeObserver:
  def __init__(self):
    self.goldRom1 = [26., 26., 26.]
    self.goldRom2 = [52., 52., 52.]

  def __call__(self, timeStep, time, state):
    print("\ntime step = {}, romState = {}".format(timeStep, state))
    if(timeStep==1):
      print("goldRomState: {}".format(self.goldRom1))
      assert( np.allclose(state, self.goldRom1, atol=1e-13) )
    if(timeStep==2):
      print("goldRomState: {}".format(self.goldRom2))
      assert( np.allclose(state, self.goldRom2, atol=1e-13) )


#----------------------------
def test():
  '''
  here we test a masked galerkin with Euler forward
  using artificial numbers, i.e. just checking things are correct.

  we use:
  - N = 10 (mimic grid points)
  - romSize = 3
  - dt = 0.1
  - phi = all 1s
  - the fom velocity returns  f = [10 20 30 40 50 60 70 80 90 100]
  - the masker only picks elements at i=2,5,6,9
   so the masked velocity should be [30, 60, 70, 100]

  galerkin here solves: x1 = x0 + dt * f2
  where x0 = [0 0 0]
  f2 = [1 1 1  ^T  [30
        1 1 1       60
        1 1 1       70
        1 1 1]     100]

  so we should get x1 = [26. 26 26]
  so we should get x2 = x1 + dt *f2 =
                      = [52. 52 52]
  '''

  N = 10
  romSize  = 3
  Nsteps   = 2
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
  galerkinProblem = rom.galerkin.masked.ProblemForwardEuler(appObj, decoder, yRom,
                                                            yRef, masker, projector)

  # fomRecon = galerkinProblem.fomStateReconstructor()
  # # the observer is called to monitor evolution of rom_state and
  # # uses the reconstructor object to reconstruct FOM state
  rom.galerkin.advanceNSteps(galerkinProblem, yRom, 0., dt, Nsteps, OdeObserver())
