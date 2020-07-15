
import numpy as np
from scipy import linalg

from burgers1d_sparse_jacobian import Burgers1dSparseJacobian
from pressio4py import rom as rom

#----------------------------
class MyLinSolver:
  def __init__(self): pass

  # @staticmethod
  def solve(self, A,b,x):
    lumat, piv, info = linalg.lapack.dgetrf(A, overwrite_a=True)
    x[:], info = linalg.lapack.dgetrs(lumat, piv, b, 0, 0)

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
  # load basis
  phi = np.loadtxt("./burgers1d_lspg/svd_basis_ncell20_t010_dt001_implicit_euler.txt")

  # create a decoder
  decoder = rom.LinearDecoder(phi)
  # the LSPG state
  yRom = np.zeros(romSize)

  lspgObj = rom.lspg.unsteady.ProblemEuler(appObj, yRef, decoder, yRom, t0)
  stepper = lspgObj.getStepper()

  # linear and non linear solver
  lsO = MyLinSolver()
  nlsO = rom.lspg.unsteady.GaussNewton(stepper, yRom, lsO)
  nlsTol, nlsMaxIt = 1e-13, 5
  nlsO.setMaxIterations(nlsMaxIt)
  nlsO.setTolerance(nlsTol)
  # do integration
  rom.lspg.unsteady.advanceNSteps(stepper, yRom, t0, dt, Nsteps, nlsO)

  fomRecon = lspgObj.getFomStateReconstructor()
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
