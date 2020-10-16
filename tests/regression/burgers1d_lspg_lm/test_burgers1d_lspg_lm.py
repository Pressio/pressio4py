
import numpy as np
from scipy import linalg

from burgers1d_sparse_jacobian import Burgers1dSparseJacobian
from pressio4py import rom as rom
from pressio4py import ode as ode
from pressio4py import solvers as solvers

np.set_printoptions(linewidth=140)

# gold solution expected from this test
gold = np.array([
  1.20961484805614e+00,
  1.00414341203934e+00,
  1.00225747155818e+00,
  1.00247921957863e+00,
  1.00273983077854e+00,
  1.00302857017833e+00,
  1.00334574603388e+00,
  1.00369802443198e+00,
  1.00408787911375e+00,
  1.00451653245805e+00,
  1.00499222980303e+00,
  1.00551742704848e+00,
  1.00609739288540e+00,
  1.00673920319338e+00,
  1.00744706708639e+00,
  1.00823132823391e+00,
  1.00909787535280e+00,
  1.01005373889436e+00,
  1.01111169307645e+00,
  1.01228245079179e+00])

#----------------------------
class MyLinSolver:
  def __init__(self): pass

  def solve(self, A,b,x):
    lumat, piv, info = linalg.lapack.dgetrf(A, overwrite_a=True)
    # here we must use x[:] otherwise it won't overwrite x passed in
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
  basisFile = "./burgers1d_lspg/svd_basis_ncell20_t010_dt001_implicit_euler.txt"
  phi = np.loadtxt(basisFile)

  # create a decoder
  decoder = rom.Decoder(phi)
  # the LSPG state
  yRom = np.zeros(romSize)

  lspgObj = rom.lspg.unsteady.default.ProblemEuler(appObj, decoder, yRom, yRef)
  stepper = lspgObj.stepper()

  # linear and non linear solver
  lsO = MyLinSolver()
  nlsO = solvers.LevenbergMarquardt(stepper, yRom, lsO)
  nlsO.setUpdatingCriterion(solvers.update.LMSchedule1)
  nlsO.setMaxIterations(2)

  # do integration
  ode.advanceNSteps(stepper, yRom, t0, dt, Nsteps, nlsO)

  fomRecon = lspgObj.fomStateReconstructor()
  yFomFinal = fomRecon.evaluate(yRom)
  print(yFomFinal)

  for y1,y2 in zip(gold, yFomFinal):
    assert( np.abs(y1-y2) < 1e-10)
