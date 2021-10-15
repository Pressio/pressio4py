
import numpy as np
from scipy import linalg
from pressio4py import solvers, ode, rom

np.set_printoptions(linewidth=140)

#----------------------------
class MyMasker:
  def __init__(self, Nsm, romSize, indices):
    self.Nsmesh_ = Nsm
    self.romSize_ = romSize
    self.rows_ = indices

  def createApplyMaskResult(self, operand):
    if (operand.ndim == 1):
      return np.zeros(self.Nsmesh_)
    else:
      return np.zeros((self.Nsmesh_, operand.shape[1]))

  def __call__(self, operand, time, result):
    if (operand.ndim == 1):
      result[:] = np.take(operand, self.rows_)
    else:
      result[:] = np.take(operand, self.rows_, axis=0)

#----------------------------
class MyTestApp:
  def __init__(self, N):
    self.N_ = N

  def createVelocity(self):
    return np.zeros(self.N_)

  def createApplyJacobianResult(self, B):
    return np.zeros((self.N_, B.shape[1]))

  def velocity(self, u, t, f):
    assert(len(f) == self.N_)
    f[:] = np.array([-1.,1.1, -1., -1., 2.2, 3.3, 0., 4.4, 5.5, 0])

  def applyJacobian(self, u, B, t, A):
    assert(A.shape[0] == self.N_)
    A[:,:] = 0.
    A[1,:] = 2.
    A[4,:] = 3.
    A[5,:] = 4.
    A[7,:] = 5.
    A[8,:] = 6.

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
  def __call__(self, timeStep, time, state):
    print(state)
    assert(state.shape[0]==3)

#----------------------------
def test():
  '''
  check that he masked bdf1 lspg problem works as expected.
  this test should do the same as unsteady_lspg_hyperreduced_trivialapp_bdf1
  except that we use a mask here to mimic hyperreduction

  We don't solve a real problem but we we use fake data and veryify that
  at every stage involved, from constructor to fom querying to solve,
  the data is supposed to be correct.
  We have the following:
  - full mesh: 10 points enumerated as {0,1,...,9}
        o  o  o  o  o  o  o  o  o  o  (points)
        0  1  2  3  4  5  6  7  8  9  (indices of sample mesh points)

  - sample mesh: subset of mesh points {1,4,5,7,8}
           *        *  *     *  *
           0        4  5     7  8  (indices wrt full mesh)
           0        1  2     3  4  (enumaration wrt to sample mesh only)

  - romSize = 3
  - start from romState = [0,0,0]
  - reference Fom state is null, i.e. all zeros
  - use GN with normal equations and do two iterations of the GN solver,
    inner linear solve: simply sets correction to be [1 1 ... 1]
  - linear mapping such that:
        phi = [ 1 1 1;
                ...  ;
              10 10 10]

  - we do one time steps: t0 -> t1
  - dt fixed at 0.2
  - the fomObj returns the velocity on the full grid points:
        f = [-1 1.1 -1 -1 2.2 3.3 0 4.4 5.5 0]
  - and returns the applyJac which has size (Npoints, romSize)
    applyJac = [0 0 0;
                2 2 2;
                0 0 0;
                0 0 0;
                3 3 3;
                4 4 4;
                0 0 0;
                5 5 5;
                6 6 6;
                0 0 0]

  - the mask extracts only the samples mesh rows, so 1,4,5,7,8

  =========================
  =========================
        time step 1
  =========================
  =========================

  **********************************************
  *** first call to nonlinear solver we have ***
  **********************************************
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

  so that the linear solver should have:
  b = -lspgJac^T R = [ 20.46 20.46 20.46 ]
  neg sign because of the sign convention in pressio

  A = (lspgJac)^T (lspgJac) =
        [158.8 158.8 158.8;
        158.8 158.8 158.8;
        158.8 158.8 158.8]

  **********************************************
  *** second call to nonlin solver we have ***
  **********************************************
  romState     = [1 1 1]
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

  so that the linear solver should have:
  b = -lspgJac^T R = [ -527.34 -527.34 -527.34 ]
  the neg sign because of the sign convention in pressio

  A = (lspgJac)^T (lspgJac) =
  [158.8 158.8 158.8;
   158.8 158.8 158.8;
   158.8 158.8 158.8]
  '''

  Ngrid   = 10
  Nsmesh  = 5
  romSize = 3

  indices = [1,4,5,7,8]
  masker = MyMasker(Nsmesh, romSize, indices)

  appObj  = MyTestApp(Ngrid)
  yRef    = np.zeros(Ngrid)
  phi     = np.zeros((Ngrid, romSize), order='F')
  for i in range(Ngrid): phi[i,:] = float(i+1)

  decoder = rom.Decoder(phi)
  yRom    = np.zeros(romSize)
  scheme = ode.stepscheme.BDF1
  problem = rom.lspg.unsteady.MaskedProblem(scheme,appObj,decoder,yRom,yRef,masker)

  # linear and non linear solver
  lsO = MyLinSolver()
  nlsO = solvers.create_gauss_newton(problem, yRom, lsO)
  nlsO.setUpdatingCriterion(solvers.update.Standard)
  nlsO.setMaxIterations(2)
  nlsO.setStoppingCriterion(solvers.stop.AfterMaxIters)

  # solve
  myObs = OdeObserver()
  ode.advance_n_steps_and_observe(problem, yRom, 0.,0.2, 1, myObs, nlsO)

  # yRom should be [1 1 1]
  fomRecon = problem.fomStateReconstructor()
  yFomFinal = fomRecon(yRom)
  np.set_printoptions(precision=15)

  # the goldFomState = phi*[1 1 1]
  goldFomState = [3, 6, 9, 12, 15, 18, 21, 24, 27, 30]
  assert( np.allclose(yFomFinal, goldFomState, atol=1e-13) )
