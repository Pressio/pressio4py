
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
class MyQRSolver:
  def __init__(self): pass

  def computeThin(self, A):
    print("computeThin ", A.shape)
    pass

  def applyQTranspose(self, operand, result):
    print("applyQTranspose ", operand.shape, result.shape)
    pass

  def applyRTranspose(self, operand, result):
    print("applyRTranspose ", operand.shape, result.shape)
    pass

  def solveRxb(self, operand, result):
    print("solveRxb ", operand.shape, result.shape)
    pass

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
  phi = np.ones((meshSize, romSize))
  phi[:,0] = 1
  phi[:,1] = 2
  phi[:,2] = 3

  # decoder
  decoder = rom.Decoder(phi)
  # LSPG state
  yRom = np.zeros(romSize)
  # create LSPG problem
  lspgProblem = rom.lspg.steady.default.Problem(appObj, decoder, yRom, yRef)

  # qr and non linear solver
  qrS = MyQRSolver()
  nlsO = solvers.GaussNewtonQR(lspgProblem, yRom, qrS)

  nlsMaxIt = 1
  nlsO.setMaxIterations(nlsMaxIt)
  # solve
  rom.lspg.solveSteady(lspgProblem, yRom, nlsO)

  # print(yRom)
  # assert(yRom[0]==3.)
  # assert(yRom[1]==6.)
  # assert(yRom[2]==9.)

  # fomRecon = lspgProblem.fomStateReconstructor()
  # yFomFinal = fomRecon.evaluate(yRom)
  # print(yFomFinal)
  # # gold is 43 because we have phi*rho=42 + reference state
  # gold = [43.,43.,43.,43.,43.,43.]
  # for y1,y2 in zip(gold, yFomFinal):
  #   assert(y1==y2)
