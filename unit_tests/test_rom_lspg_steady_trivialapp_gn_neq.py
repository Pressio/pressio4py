
import numpy as np
from scipy import linalg
from pressio4py import logger, solvers, rom
from pressio4py.rom import lspg

class MySteadyAdapter:
  def __init__(self, N):
    assert(N==6)
    self.N_ = N

  def createResidual(self):
    return np.zeros(self.N_)

  def createApplyJacobianResult(self, operand):
    return np.zeros_like(operand)

  def residual(self, stateIn, R):
    R[:] = 1.0

  def applyJacobian(self, stateIn, operand, C):
    J = self.jacobian(stateIn)
    C[:]  = J.dot(operand)

  def jacobian(self, stateIn):
    return np.identity(self.N_)

#----------------------------
class MyLinSolver:
  def solve(self, A,b,x):
    assert(A[0,0]==6);assert(A[0,1]==12);assert(A[0,2]==18)
    assert(A[1,0]==12);assert(A[1,1]==24);assert(A[1,2]==36)
    assert(A[2,0]==18);assert(A[2,1]==36);assert(A[2,2]==54)
    # here I would need to solve the system, but for testing
    # let's say we fix the solution. Rememeber this is the correction
    # of the inner linear solve inside the GN
    assert(len(x)==3)
    x[:] = np.array([1.,2.,3.])

#----------------------------
def test_steady_lspg():
  logger.initialize(logger.logto.terminal)
  logger.setVerbosity([logger.loglevel.debug])

  np.random.seed(334346892)

  meshSize = 6
  romSize = 3
  appObj = MySteadyAdapter(meshSize)
  yRef = np.ones(meshSize)
  phi = np.ones((meshSize, romSize), order='F')
  phi[:,0] = 1
  phi[:,1] = 2
  phi[:,2] = 3

  decoder = rom.Decoder(phi)
  yRom = np.zeros(romSize)
  problem = lspg.steady.Problem(appObj, decoder, yRom, yRef)

  # linear and non linear solver
  lsO = MyLinSolver()
  nlsO = solvers.create_gauss_newton(problem, yRom, lsO)

  nlsTol, nlsMaxIt = 1e-13, 3
  nlsO.setMaxIterations(nlsMaxIt)
  nlsO.setTolerance(nlsTol)
  assert(nlsO.correctionAbsoluteTolerance()==1e-13)
  assert(nlsO.maxIterations()==3)
  # solve
  nlsO.solve(problem, yRom)

  print(yRom)
  assert(yRom[0]==3.)
  assert(yRom[1]==6.)
  assert(yRom[2]==9.)

  fomRecon = problem.fomStateReconstructor()
  yFomFinal = fomRecon(yRom)
  print(yFomFinal)
  # gold is 43 because we have phi*rho=42 + reference state
  gold = [43.,43.,43.,43.,43.,43.]
  for y1,y2 in zip(gold, yFomFinal):
    assert(y1==y2)

#----------------------------
class MyPrec:
  def __call__(self, state, operand):
    print(state.shape)
    operand[:] *= 1.

#----------------------------
def test_steady_lspg_prec():
  logger.initialize(logger.logto.terminal)
  logger.setVerbosity([logger.loglevel.debug])
  np.random.seed(334346892)
  meshSize = 6
  romSize = 3
  appObj = MySteadyAdapter(meshSize)
  yRef = np.ones(meshSize)
  phi = np.ones((meshSize, romSize), order='F')
  phi[:,0] = 1
  phi[:,1] = 2
  phi[:,2] = 3

  decoder = rom.Decoder(phi)
  yRom = np.zeros(romSize)
  prec = MyPrec()
  problem = lspg.steady.PrecProblem(appObj, decoder, yRom, yRef, prec)

  # linear and non linear solver
  lsO = MyLinSolver()
  nlsO = solvers.create_gauss_newton(problem, yRom, lsO)

  nlsTol, nlsMaxIt = 1e-13, 3
  nlsO.setMaxIterations(nlsMaxIt)
  nlsO.setTolerance(nlsTol)
  assert(nlsO.correctionAbsoluteTolerance()==1e-13)
  assert(nlsO.maxIterations()==3)
  # solve
  nlsO.solve(problem, yRom)

  print(yRom)
  assert(yRom[0]==3.)
  assert(yRom[1]==6.)
  assert(yRom[2]==9.)

  fomRecon = problem.fomStateReconstructor()
  yFomFinal = fomRecon(yRom)
  print(yFomFinal)
  # gold is 43 because we have phi*rho=42 + reference state
  gold = [43.,43.,43.,43.,43.,43.]
  for y1,y2 in zip(gold, yFomFinal):
    assert(y1==y2)

  logger.finalize()
