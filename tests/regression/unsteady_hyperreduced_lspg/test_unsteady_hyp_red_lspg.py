
import numpy as np
from scipy import linalg

from pressio4py import rom as rom
from pressio4py import solvers as solvers

np.set_printoptions(linewidth=140)

class MyTestApp:
  def __init__(self, Nst, Nsm, romSize):
    self.Nstencil_ = Nst
    self.Nsmesh_ = Nsm
    self.romSize_ = romSize

  def createVelocity(self):
    return np.zeros(self.Nsmesh_)

  def createApplyJacobianResult(self, B):
    return np.zeros((self.Nsmesh_, B.shape[1]))

  def velocity(self, u, t, f):
    assert(len(f) == self.Nsmesh_)
    f[:] = np.array([1.1, 2.2, 3.3, 4.4, 5.5])

  def applyJacobian(self, u, B, t, A):
    assert(A.shape[0] == self.Nsmesh_)
    assert(A.shape[1] == self.romSize_)
    assert(self.Nsmesh_ == 5)
    A[0,:] = 2.;
    A[1,:] = 3.;
    A[2,:] = 4.;
    A[3,:] = 5.;
    A[4,:] = 6.;

#----------------------------
class MyLinSolver:
  def __init__(self):
    self.callCount_ = 0

  def solve(self, A,b,x):
    self.callCount_ += 1
    x[:] = 1.

    if self.callCount_ == 1:
      bGold = np.ones(3)*(20.46)
      assert( np.allclose(b, bGold, atol=1e-12) )
      hGold = np.ones((3,3))*158.8
      assert( np.allclose(A, hGold, atol=1e-12) )

    if self.callCount_ == 2:
      bGold = np.ones(3)*(-527.34)
      assert( np.allclose(b, bGold, atol=1e-12) )
      hGold = np.ones((3,3))*158.8
      assert( np.allclose(A, hGold, atol=1e-12) )


#----------------------------------------
class OdeObserver:
  def __init__(self): pass

  def __call__(self, timeStep, time, state):
    print(state)
    assert(state.shape[0]==3)

#----------------------------
def test():
  '''
  this test checks that the hyp-red lspg problem works as expected.
  We don't solve a real problem but we we use fake data and veryify that
  at every stage involved, from constructor to fom querying to solve,
  the data is supposed to be correct.
  We have the following:
  - stencils mesh: 10 points enumerated as {0,1,...,9}
        o  o  o  o  o  o  o  o  o  o  (points)
        0  1  2  3  4  5  6  7  8  9  (indices of sample mesh points)

  - sample mesh: subset of stencils mesh points {1,4,5,7,8}
        *        *  *     *  *
        0        4  5     7  8  (indices wrt stencil mesh)
        0        1  2     3  4  (enumaration wrt to sample mesh only)

  - romSize = 3
  - start from romState = [0,0,0]
  - use GN with normal equations and do two iterations of the GN solver,
    inner linear solve: simply sets correction to be [1 1 ... 1]
  - linear mapping such that:
              phi = [ 1 1 1;
                      ...  ;
                     10 10 10]

  - we do one time steps: t0 -> t1, with dt = 0.2
  - the fomObj returns the velocity at the sample mesh points:
          f = [1.1 2.2 3.3 4.4 5.5]
  - the fomObj returns the applyJac which has size (Nsmesh, romSize)
    applyJac = [2 2 2;
                3 3 3;
                4 4 4;
                5 5 5;
                6 6 6]

  =========================
  =========================
        time step 1
  =========================
  =========================

  *************************************
  *** first call to solver we have ***
  *************************************
  romState     = [0 0 0],
  fomState_n   = [0 ... 0]
  fomState_n-1 = [0 ... 0]

  R = y_n - y_nm-1 - dt*f
    = [-0.22 -0.44 -0.66 -0.88 -1.1]

  lspgJac = I*phi - dt*df/dy*phi
  where I*phi is only taken at the sample mesh points
  and df/dy*phi = [2 2 2;
                   3 3 3;
                   4 4 4;
                   5 5 5;
                   6 6 6]
  we get:
  lspgJac = [2-dt*2 2-dt*2 2-dt*2;    [1.6 1.6 1.6;
             5-dt*3 5-dt*3 5-dt*3;     4.4 4.4 4.4;
             6-dt*4 6-dt*4 6-dt*4; =   5.2 5.2 5.2;
             8-dt*5 8-dt*5 8-dt*5;     7.0 7.0 7.0;
             9-dt*6 9-dt*6 9-dt*6]     7.8 7.8 7.8]

  so that the first call to the linear solver should have:
  b = -lspgJac^T R = [ 20.46 20.46 20.46 ]
  neg sign because of the sign convention in pressio

  A = (lspgJac)^T (lspgJac) =
        [158.8 158.8 158.8;
        158.8 158.8 158.8;
        158.8 158.8 158.8]


  *************************************
  *** second call to solver we have ***
  *************************************
  romState     = [1 1 1],
  fomState_n   = [3 6 9 12 15 18 21 24 27 30]
  fomState_n-1 = [0 ... 0]

  R = y_n - y_nm-1 - dt*f
   = [6-0.22 15-0.44 18-0.66 24-0.88 27-1.1]
   = [5.78 14.56 17.34 23.12 25.9]

  lspgJac = I*phi - dt*df/dy*phi
  where I*phi is only taken at the sample mesh points
  and df/dy*phi = [2 2 2;
                   3 3 3;
                   4 4 4;
                   5 5 5;
                   6 6 6]
  we get:
  lspgJac = [2-dt*2 2-dt*2 2-dt*2;    [1.6 1.6 1.6;
             5-dt*3 5-dt*3 5-dt*3;     4.4 4.4 4.4;
             6-dt*4 6-dt*4 6-dt*4; =   5.2 5.2 5.2;
             8-dt*5 8-dt*5 8-dt*5;     7.0 7.0 7.0;
             9-dt*6 9-dt*6 9-dt*6]     7.8 7.8 7.8]

  so that the first call to the linear solver should have:
  b = -lspgJac^T R = [ -527.34 -527.34 -527.34 ]
  the neg sign because of the sign convention in pressio

  A = (lspgJac)^T (lspgJac) =
  [158.8 158.8 158.8;
   158.8 158.8 158.8;
   158.8 158.8 158.8]
  '''

  Nstencil = 10
  Nsmesh   = 5
  romSize = 3

  appObj  = MyTestApp(Nstencil, Nsmesh, romSize)
  yRef    = np.ones(Nstencil)
  phi     = np.zeros((Nstencil, romSize))
  for i in range(Nstencil): phi[i,:] = float(i+1)

  decoder = rom.Decoder(phi)
  yRom    = np.zeros(romSize)

  indices = [1,4,5,7,8]
  lspgProblem = rom.lspg.unsteady.hyperreduced.ProblemEuler(appObj, decoder,
                                                            yRom, yRef, indices)

  # linear and non linear solver
  lsO = MyLinSolver()
  nlsO = solvers.GaussNewton(lspgProblem, yRom, lsO)
  nlsO.setUpdatingCriterion(solvers.update.standard)
  nlsO.setMaxIterations(2)
  nlsO.setStoppingCriterion(solvers.stop.afterMaxIters)

  # do integration
  myObs = OdeObserver()
  rom.lspg.solveNSequentialMinimizations(lspgProblem, yRom, 0.,
                                         0.2, 1, myObs, nlsO)

  # yRom should be [1 1 1]
  fomRecon = lspgProblem.fomStateReconstructor()
  yFomFinal = fomRecon.evaluate(yRom)
  np.set_printoptions(precision=15)

  # the goldFomState = phi*[1 1 1] + yRef
  goldFomState = [4,7, 10, 13, 16, 19, 22, 25, 28, 31]
  assert( np.allclose(yFomFinal, goldFomState, atol=1e-13) )
