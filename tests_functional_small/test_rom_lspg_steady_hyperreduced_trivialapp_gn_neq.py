
import numpy as np
from scipy import linalg
from pressio4py import logger, solvers, rom

class MySteadyAdapter:
  def __init__(self, N):
    self.N = N
    self.indices = [0,2,4,6,8,10,12,14]

  def createResidual(self):
    return np.zeros(self.N)

  def createApplyJacobianResult(self, operand):
    return np.zeros((self.N, operand.shape[1]))

  def residual(self, stateIn, R):
    assert(len(stateIn) == 15)
    assert(len(stateIn) != len(R))
    assert(len(R) == self.N)

    for i in range(len(self.indices)):
      R[i] = stateIn[self.indices[i]] + 1.
      print("R:: ", i, R[i])

  def applyJacobian(self, stateIn, B, A):
    for i in range(len(self.indices)):
      A[i,0] = B[self.indices[i],0]
      A[i,1] = B[self.indices[i],1]
      A[i,2] = B[self.indices[i],2]

    A[:] += 1.


class MyNonLinSolver:

  def solve(self, system, state):
    R = system.createResidual()
    J = system.createJacobian()

    system.residual(state, R)
    system.jacobian(state, J)
    assert(R[0] == 0*0.+1*1.+2*2.+1.)
    assert(R[1] == 3*0.+4*1.+5*2.+1.)
    assert(R[2] == 6*0.+7*1.+8*2.+1.)
    assert(R[3] == 9*0.+10*1.+11*2.+1.)
    assert(R[4] == 12*0.+13*1.+14*2.+1.)
    assert(R[5] == 15*0.+16*1.+17*2.+1.)
    assert(R[6] == 18*0.+19*1.+20*2.+1.)
    assert(R[7] == 21*0.+22*1.+23*2.+1.)

    start = 1
    count = 0
    for i in range(8):
      for j in range(3):
        assert(J[i,j] == start + float(count))
        count += 1

    state[:] += 1

    system.residual(state, R)
    system.jacobian(state, J)
    assert(R[0] == 0*1.+1*2.+2*3.+1.)
    assert(R[1] == 3*1.+4*2.+5*3.+1.)
    assert(R[2] == 6*1.+7*2.+8*3.+1.)
    assert(R[3] == 9*1.+10*2.+11*3.+1.)
    assert(R[4] == 12*1.+13*2.+14*3.+1.)
    assert(R[5] == 15*1.+16*2.+17*3.+1.)
    assert(R[6] == 18*1.+19*2.+20*3.+1.)
    assert(R[7] == 21*1.+22*2.+23*3.+1.)

    start = 1
    count = 0
    for i in range(8):
      for j in range(3):
        assert(J[i,j] == start + float(count))
        count += 1

    state[:] += 1

#----------------------------
def testrun():
  nstencil = 15;
  nSample  = 8;
  appObj = MySteadyAdapter(nSample)

  fomReferenceState = np.zeros(nstencil)

  phi = np.zeros((nstencil, 3), order='F')
  count = 0
  for i in range(nstencil):
    for j in range(3):
      if (i % 2 == 0):
        phi[i,j] = float(count)
        count+=1
      else:
        phi[i,j] = -1.


  decoder = rom.Decoder(phi)
  yRom = np.array([0., 1., 2.])

  lspgProblem = rom.lspg.steady.Problem(appObj, decoder, yRom, fomReferenceState)
  solv = MyNonLinSolver()
  solv.solve(lspgProblem, yRom)

  assert(yRom[0]==2.)
  assert(yRom[1]==3.)
  assert(yRom[2]==4.)
