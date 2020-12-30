
import numpy as np
from scipy import linalg

from pressio4py import rom as rom
from pressio4py import solvers as solvers

class MySteadyAdapter:
  def __init__(self, N):
    assert(N==6)
    self.N_     = N

  def createResidual(self):
    print("DemoSteadyLSPG: createResidual")
    return np.zeros(self.N_)

  def createApplyJacobianResult(self, operand):
    print("DemoSteadyLSPG: createApplyJacobianResult")
    return np.zeros_like(operand)

  def residual(self, stateIn, R):
    print("DemoSteadyLSPG: residual")
    R[:] = 1.0

  def applyJacobian(self, stateIn, operand, C):
    J = self.jacobian(stateIn)
    print("DemoSteadyLSPG: applyingJacobian")
    C[:]  = J.dot(operand)

  def jacobian(self, stateIn):
    return np.identity(self.N_)

#----------------------------
class MyLinSolver:
  def __init__(self): pass

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
  np.random.seed(334346892)

  meshSize = 6
  romSize = 3

  # create app
  appObj = MySteadyAdapter(meshSize)
  # set reference state
  yRef = np.ones(meshSize)
  # load basis
  phi = np.ones((meshSize, romSize), order='F')
  phi[:,0] = 1
  phi[:,1] = 2
  phi[:,2] = 3

  # decoder
  decoder = rom.Decoder(phi)
  # LSPG state
  yRom = np.zeros(romSize)
  # create LSPG problem
  lspgProblem = rom.lspg.steady.default.Problem(appObj, decoder, yRom, yRef)

  # linear and non linear solver
  lsO = MyLinSolver()
  nlsO = solvers.GaussNewton(lspgProblem, yRom, lsO)

  nlsTol, nlsMaxIt = 1e-13, 3
  nlsO.setMaxIterations(nlsMaxIt)
  nlsO.setTolerance(nlsTol)
  assert(nlsO.correctionAbsoluteTolerance()==1e-13)
  assert(nlsO.maxIterations()==3)
  # solve
  rom.lspg.solveSteady(lspgProblem, yRom, nlsO)

  print(yRom)
  assert(yRom[0]==3.)
  assert(yRom[1]==6.)
  assert(yRom[2]==9.)

  fomRecon = lspgProblem.fomStateReconstructor()
  yFomFinal = fomRecon.evaluate(yRom)
  print(yFomFinal)
  # gold is 43 because we have phi*rho=42 + reference state
  gold = [43.,43.,43.,43.,43.,43.]
  for y1,y2 in zip(gold, yFomFinal):
    assert(y1==y2)
