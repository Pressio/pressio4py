
import numpy as np
from scipy import linalg

# need to add to python path location of the apps
import pathlib, sys
file_path = pathlib.Path(__file__).parent.absolute()
sys.path.append(str(file_path) + "/../apps")

from burgers1d_sparse_jacobian import Burgers1dSparseJacobian
from pressio4py import rom as rom
from pressio4py import solvers as solvers

#----------------------------
class MyLinSolver:
  def __init__(self): pass

  # @staticmethod
  def solve(self, A,b,x):
    lumat, piv, info = linalg.lapack.dgetrf(A, overwrite_a=True)
    x[:], info = linalg.lapack.dgetrs(lumat, piv, b, 0, 0)

#----------------------------
class MyMapper:
  def __init__(self):
    fname = str(file_path) + "/basis_euler.txt"
    # I have to make phi a column-major array to ensure
    # pressio does not make a copy of this
    self.phi_ = np.copy(np.loadtxt(fname), order='F')
    phi_addr = self.phi_.__array_interface__['data'][0]
    print("map:phi: ", hex(phi_addr))

  def jacobian(self):
    return self.phi_

  def applyMapping(self, romState, fomState):
    fomState[:] = self.phi_.dot(romState)

  def updateJacobian(self, romState):
    # do nothing here for this test
    pass

#----------------------------------------
class OdeObserver:
  def __init__(self): pass

  def __call__(self, timeStep, time, state):
    print(state)
    assert(state.shape[0]==11)

#----------------------------
def test_euler():
  meshSize = 20
  romSize  = 11
  Nsteps   = 10
  dt       = 0.01
  t0       = 0.

  # create app
  appObj = Burgers1dSparseJacobian(meshSize)
  # set reference state
  yRef = np.ones(meshSize)

  # decoder
  mymap   = MyMapper()
  # needs a description string
  decoder = rom.Decoder(mymap, "MyMapper")
  print("dec:add: ", hex(decoder.jacobianAddress()))
  # LSPG state
  yRom = np.zeros(romSize)

  lspgProblem = rom.lspg.unsteady.default.ProblemEuler(appObj, decoder, yRom, yRef)

  # linear and non linear solver
  lsO = MyLinSolver()
  nlsO = solvers.GaussNewton(lspgProblem, yRom, lsO)
  nlsTol, nlsMaxIt = 1e-13, 4
  nlsO.setMaxIterations(nlsMaxIt)
  nlsO.setStoppingCriterion(solvers.stop.whenCorrectionAbsoluteNormBelowTolerance)
  nlsO.setCorrectionAbsoluteTolerance(nlsTol)

  # solve
  myObs = OdeObserver()
  rom.lspg.solveNSequentialMinimizations(lspgProblem, yRom, t0, dt, Nsteps, myObs, nlsO)

  fomRecon = lspgProblem.fomStateReconstructor()
  yFomFinal = fomRecon.evaluate(yRom)
  print(yFomFinal)

  gold = np.array([1.2392405345107, 1.0051378268469,
                   1.0025875046782, 1.0028353031206,
                   1.0031333374311, 1.0034628717396,
                   1.0038270641633, 1.0042295588277,
                   1.0046743839626, 1.0051659914443,
                   1.0057093013441, 1.0063097511659,
                   1.0069733502617, 1.0077067399692,
                   1.0085172600729, 1.0094130222541,
                   1.0104029912645, 1.0114970746344,
                   1.0127062218147, 1.0140425337419])

  for y1,y2 in zip(gold, yFomFinal):
    assert( np.abs(y1-y2) < 1e-10)
