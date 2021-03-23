
import numpy as np
from scipy import linalg

from pressio4py import rom as rom
from pressio4py import solvers as solvers

class MySteadyAdapter:
  def __init__(self, N):
    self.N_ = N

  def createResidual(self):
    return np.zeros(self.N_)

  def createApplyJacobianResult(self, operand):
    return np.zeros_like(operand)

  def residual(self, stateIn, R):
    R[:] = np.arange(self.N_)

  def applyJacobian(self, stateIn, operand, C):
    C[:,0] = np.arange(self.N_)
    C[:,1] = np.arange(self.N_)
    C[:,2] = np.arange(self.N_)

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
class MyMasker:
  def __init__(self, indices):
    self.rows_ = indices
    self.sampleMeshSize_ = len(indices)

  def createApplyMaskResult(self, operand):
    if (operand.ndim == 1):
      return np.zeros(self.sampleMeshSize_)
    else:
      return np.zeros((self.sampleMeshSize_, operand.shape[1]))

  def applyMask(self, operand, result):
    result[:] = np.take(operand, self.rows_, axis=0)

#----------------------------
def testrun():
  '''
  phi =
  [0,0,0,
   1,1,1,
   2,2,2,
   3,3,3,
   4,4,4,
   5,5,5,
   6,6,6
   7,7,7]

  fomApp:
   residual = [0,1,2,3,4,5,6,7]

   applyJacobian =
                   [0,0,0,
                    1,1,1,
                    2,2,2,
                    3,3,3,
                    4,4,4,
                    5,5,5,
                    6,6,6
                    7,7,7]

   applyMasker picks the rows = 0,1,3,4,6

  solve using normal eq which should give:

   H x = b

   H = [62,62,62,
        62,62,62,
        62,62,62]

  b = -62,-62,-62
  '''

  np.random.seed(334346892)

  meshSize = 7
  romSize = 3

  appObj = MySteadyAdapter(meshSize)
  yRef = np.ones(meshSize)
  phi = np.ones((meshSize, romSize), order='F')
  decoder = rom.Decoder(phi)
  yRom = np.zeros(romSize)

  sampleMeshIndices = [0,1,3,4,6]
  masker = MyMasker(sampleMeshIndices)

  lspgProblem = rom.lspg.steady.masked.Problem(appObj, decoder, yRom, yRef, masker)
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
