
import numpy as np
from scipy import linalg

# need to add to python path location of the apps
import pathlib, sys
file_path = pathlib.Path(__file__).parent.absolute()
sys.path.append(str(file_path) + "/../apps")

from burgers1d_sparse_jacobian import Burgers1dSparseJacobian
from pressio4py import rom as rom

np.set_printoptions(linewidth=140)

class MyTestApp:
  def __init__(self, N, numFields):
    self.N_ = N
    self.numFields_ =numFields

  def createVelocity(self):
    return np.zeros((self.N_, self.numFields_))

  def velocity(self, u, t, f):
    assert(self.numFields_==2)
    assert(self.N_==8)

    print(f.flags['C_CONTIGUOUS'])
    print(f.flags['F_CONTIGUOUS'])
    print(f.shape)
    assert(f.shape[0] == self.N_)
    assert(f.shape[1] == self.numFields_)
    f[:,0] = np.copy(np.array([1.,2.,3.,4.,5.,6.,7.,8.]))
    f[:,1] = np.copy(np.array([2.,3.,4.,5.,6.,7.,8.,9.]))

#----------------------------------------
class OdeObserver:
  def __init__(self, fomRec):
    self.fomRec = fomRec

  def __call__(self, timeStep, time, state):
    print(state)
    fs = self.fomRec.evaluate(state)
    print(fs.shape)
    assert(fs.shape[0]==50)

#----------------------------------------
def test_euler():
  '''
  here we test galerkin for when we have an app with
  multiple fields: this means the fom_state and the rom state
  are set as rank-2 numpy arrays and each column
  corresponds to a field.
  By fields, here we mean physical quantities, or dofs.

  The decoder's jacobian is a rank-3 array.


  For the rank-3 basis, the k-th slice along third axis
  represents the rank-2 basis for the k-th field.

  Instead of relying on a reall app, we fake things,
  just interested in checking numerics is correct.
  '''

  numFields = 2
  meshSize = 8
  romSize  = 3
  Nsteps   = 1
  dt       = 2.

  # create fom object
  appObj = MyTestApp(meshSize, numFields)
  # reference state
  yRef = np.ones((meshSize, numFields), order='F')

  phi = np.zeros((meshSize, romSize, numFields), order='F')
  phi[:,:,0] = 1.
  phi[:,:,1] = 2.
  decoder = rom.rank2state.MultiFieldDecoder(phi)

  # create rom state
  yRom = np.zeros((romSize, numFields), order='F')
  # create problem
  galerkinProblem = rom.galerkin.multifield.default.ProblemForwardEuler(appObj,
                                                                        decoder,
                                                                        yRom, yRef)

  # fomRecon = galerkinProblem.fomStateReconstructor()
  # # the observer is called to monitor evolution of rom_state and
  # # uses the reconstructor object to reconstruct FOM state
  # myObs = OdeObserver(fomRecon)
  rom.galerkin.advanceNSteps(galerkinProblem, yRom, 0., dt, Nsteps)

  goldRom = np.array([[72.,176.], [72.,176.], [72.,176.]])
  assert(np.allclose(goldRom, yRom))
  print("yRom", yRom)
