
import numpy as np
from scipy import linalg
import pathlib, sys
file_path = pathlib.Path(__file__).parent.absolute()

from pressio4py import logger, solvers, ode, rom
from pressio4py.apps.burgers1d import Burgers1d

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
class MyQRSolver:
  def __init__(self, meshSize, romSize):
    self.Q_ = np.zeros((meshSize, romSize))
    self.R_ = np.zeros((romSize, romSize))

  def computeThin(self, A):
    self.Q_, self.R_ = np.linalg.qr(A, mode='reduced')

  def applyQTranspose(self, operand, result):
    result[:] = self.Q_.T.dot(operand)

  def applyRTranspose(self, operand, result):
    result[:] = self.R_.T.dot(operand)

  def solveRxb(self, b, x):
    x[:] = linalg.solve(self.R_, b)

#----------------------------------------
class OdeObserver:
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
  appObj = Burgers1d(meshSize)

  # set reference state
  yRef = np.ones(meshSize)

  # I have to make phi a column-major array to ensure
  # pressio does not make a copy of this
  basisFile = str(file_path) + "/basis_bdf1.txt"
  phi = np.copy(np.loadtxt(basisFile), order='F')

  # decoder
  decoder = rom.Decoder(phi)
  # LSPG state
  yRom = np.zeros(romSize)
  # lspg problem
  scheme = ode.stepscheme.BDF1
  problem = rom.lspg.unsteady.DefaultProblem(scheme, appObj, decoder, yRom, yRef)

  # qr and non linear solver
  qrS = MyQRSolver(meshSize, romSize)
  nlsO = solvers.create_gauss_newton_qr(problem, yRom, qrS)
  nlsO.setUpdatingCriterion(solvers.update.Standard)
  nlsO.setMaxIterations(2)
  nlsO.setStoppingCriterion(solvers.stop.AfterMaxIters)

  # solve
  myObs = OdeObserver()
  ode.advance_n_steps_and_observe(problem, yRom, t0,dt, Nsteps, myObs, nlsO)

  fomRecon = problem.fomStateReconstructor()
  yFomFinal = fomRecon(yRom)
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

  logger.initialize(logger.logto.terminal, "null")
  logger.setVerbosity([logger.loglevel.info])

  # create app
  appObj = Burgers1d(meshSize)

  # set reference state
  yRef = np.ones(meshSize)
  # load basis
  basisFile = str(file_path) + "/basis_bdf2.txt"
  phi = np.copy(np.loadtxt(basisFile), order='F')

  # decoder
  decoder = rom.Decoder(phi)
  # LSPG state
  yRom = np.zeros(romSize)
  # lspg problem
  scheme = ode.stepscheme.BDF2
  problem = rom.lspg.unsteady.DefaultProblem(scheme, appObj, decoder, yRom, yRef)

  # qr and non linear solver
  qrS = MyQRSolver(meshSize, romSize)
  nlsO = solvers.create_gauss_newton_qr(problem, yRom, qrS)
  nlsO.setUpdatingCriterion(solvers.update.Standard)
  nlsO.setMaxIterations(2)
  nlsO.setStoppingCriterion(solvers.stop.AfterMaxIters)

  # solve
  myObs = OdeObserver()
  ode.advance_n_steps_and_observe(problem, yRom, t0,dt, Nsteps, myObs, nlsO)

  fomRecon = problem.fomStateReconstructor()
  yFomFinal = fomRecon(yRom)
  np.set_printoptions(precision=15)
  print(yFomFinal)

  for y1,y2 in zip(goldBdf2, yFomFinal):
    assert( np.abs(y1-y2) < 1e-10)

  logger.finalize()
