
import numpy as np
from scipy import linalg

import pathlib, sys
file_path = pathlib.Path(__file__).parent.absolute()
sys.path.append(str(file_path) + "/../apps")

from burgers1d_sparse_jacobian import Burgers1dSparseJacobian
from pressio4py import rom as rom
from pressio4py import solvers as solvers

np.set_printoptions(linewidth=140)

goldBdf1 = np.array([
  1.23924618529016e+00,
  1.00513224142691e+00,
  1.00258744050401e+00,
  1.00283530274169e+00,
  1.00313333759789e+00,
  1.00346287207285e+00,
  1.00382706443916e+00,
  1.00422955914887e+00,
  1.00467438433032e+00,
  1.00516599192553e+00,
  1.00570930192065e+00,
  1.00630975185226e+00,
  1.00697335112295e+00,
  1.00770674109302e+00,
  1.00851726134268e+00,
  1.00941302387047e+00,
  1.01040299320197e+00,
  1.01149707707594e+00,
  1.01270622473792e+00,
  1.01404253728095e+00])

goldBdf2 = np.array([
  1.23947296257898,
  1.004911840673181,
  1.002583292436882,
  1.002835486018458,
  1.003133591274698,
  1.003463152789714,
  1.003827374922797,
  1.004229902417551,
  1.004674763912067,
  1.005166411588689,
  1.005709766002342,
  1.006310265072025,
  1.006973918660976,
  1.0077073687177,
  1.008517955621836,
  1.009413791794709,
  1.01040384277086,
  1.011498016939365,
  1.012707264750251,
  1.014043688232101])

#----------------------------
class MyLinSolver:
  def __init__(self): pass

  # @staticmethod
  def solve(self, A,b,x):
    lumat, piv, info = linalg.lapack.dgetrf(A, overwrite_a=True)
    # here we must use x[:] otherwise it won't overwrite x passed in
    x[:], info = linalg.lapack.dgetrs(lumat, piv, b, 0, 0)

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
  # load basis
  basisFile = "./burgers1d_lspg/svd_basis_ncell20_t010_dt001_implicit_euler.txt"
  phi = np.loadtxt(basisFile)

  # decoder
  decoder = rom.Decoder(phi)
  # LSPG state
  yRom = np.zeros(romSize)
  # lspg problem
  lspgProblem = rom.lspg.unsteady.default.ProblemEuler(appObj, decoder, yRom, yRef)

  # linear and non linear solver
  lsO = MyLinSolver()
  nlsO = solvers.GaussNewton(lspgProblem, yRom, lsO)
  nlsO.setUpdatingCriterion(solvers.update.standard)
  nlsO.setMaxIterations(2)
  nlsO.setStoppingCriterion(solvers.stop.afterMaxIters)

  # solve
  myObs = OdeObserver()
  rom.lspg.solveNSequentialMinimizations(lspgProblem, yRom, t0,
                                         dt, Nsteps, myObs, nlsO)

  fomRecon = lspgProblem.fomStateReconstructor()
  yFomFinal = fomRecon.evaluate(yRom)
  np.set_printoptions(precision=15)
  print(yFomFinal)

  for y1,y2 in zip(goldBdf1, yFomFinal):
    assert( np.abs(y1-y2) < 1e-10)


#----------------------------
def test_bdf2():
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
  basisFile = "./burgers1d_lspg/svd_basis_ncell20_t010_dt001_implicit_bdf2.txt"
  phi = np.loadtxt(basisFile)

  # decoder
  decoder = rom.Decoder(phi)
  # LSPG state
  yRom = np.zeros(romSize)
  # lspg problem
  lspgProblem = rom.lspg.unsteady.default.ProblemBDF2(appObj, decoder, yRom, yRef)

  # linear and non linear solver
  lsO = MyLinSolver()
  nlsO = solvers.GaussNewton(lspgProblem, yRom, lsO)
  nlsO.setUpdatingCriterion(solvers.update.standard)
  nlsO.setMaxIterations(2)
  nlsO.setStoppingCriterion(solvers.stop.afterMaxIters)

  # solve
  myObs = OdeObserver()
  rom.lspg.solveNSequentialMinimizations(lspgProblem, yRom, t0,
                                         dt, Nsteps, myObs, nlsO)

  fomRecon = lspgProblem.fomStateReconstructor()
  yFomFinal = fomRecon.evaluate(yRom)
  np.set_printoptions(precision=15)
  print(yFomFinal)

  for y1,y2 in zip(goldBdf2, yFomFinal):
    assert( np.abs(y1-y2) < 1e-10)
