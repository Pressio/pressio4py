
import numpy as np
from scipy import linalg

from pressio4py import rom as rom
from pressio4py import solvers as solvers

class MySteadyAdapter:
  def __init__(self, stencilN, sampleN):
    self.stencilN_ = stencilN
    self.sampleN_ = sampleN

  def createResidual(self):
    return np.zeros(self.sampleN_)

  def createApplyJacobianResult(self, operand):
    return np.zeros_like(operand)

  def residual(self, stateIn, R):
    assert(R.shape[0] == self.sampleN_)
    R[:] = np.array([0,1,3,4,6])

  def applyJacobian(self, stateIn, operand, C):
    assert(C.shape[0] == self.sampleN_)
    assert(C.shape[1] == operand.shape[1])
    assert(C.shape[1] == 3)
    C[:] = np.array([[0,0,0],
                     [1,1,1],
                     [3,3,3],
                     [4,4,4],
                     [6,6,6]])

#----------------------------
class MyLinSolver:
  def __init__(self): pass

  def solve(self, A,b,x):
    print("\n")
    print(A)
    print(b)
    Agold = np.array([[62.,62.,62.],
                      [62.,62.,62.],
                      [62.,62.,62.]])
    bgold = np.array([-62.,-62.,-62.])
    assert( np.allclose(b, bgold, 1e-13) )
    assert( np.allclose(A, Agold, 1e-13) )
    assert(len(x)==3)
    x[:] = np.array([1.,2.,3.])

#----------------------------
def testrun():
  '''
  phi =
  [0,0,0,
   1,1,1,
   3,3,3,
   4,4,4,
   6,6,6]

  fomApp:
   residual = [0,1,3,4,6]

   applyJacobian =
                   [0,0,0,
                    1,1,1,
                    3,3,3,
                    4,4,4,
                    6,6,6]

  solve using normal eq which should give:

   H x = b

   H = [62,62,62,
        62,62,62,
        62,62,62]

  b = -62,-62,-62
  '''

  np.random.seed(334346892)

  stencilSize = 10
  sampleSize = 5
  romSize = 3

  appObj = MySteadyAdapter(stencilSize, sampleSize)
  yRef = np.ones(stencilSize)
  phi = np.ones((sampleSize, romSize), order='F')
  decoder = rom.Decoder(phi)
  yRom = np.zeros(romSize)

  lspgProblem = rom.lspg.steady.hyperreduced.Problem(appObj, decoder, yRom, yRef)
  lsO = MyLinSolver()
  nlsO = solvers.createGaussNewton(lspgProblem, yRom, lsO)
  nlsO.setMaxIterations(1)
  rom.lspg.solveSteady(lspgProblem, yRom, nlsO)

  print(yRom)
  assert(yRom[0]==1.)
  assert(yRom[1]==2.)
  assert(yRom[2]==3.)


if __name__ == "__main__":
  testrun()
