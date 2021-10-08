
import numpy as np
from scipy import linalg
from pressio4py import solvers, ode, rom

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
      bGold = np.ones(3)*(-339.9)
      assert( np.allclose(b, bGold, atol=1e-12) )
      hGold = np.ones((3,3))*198
      assert( np.allclose(A, hGold, atol=1e-12) )
    if self.callCount_ == 2:
      bGold = np.ones(3)*(263.1)
      assert( np.allclose(b, bGold, atol=1e-12) )
      hGold = np.ones((3,3))*198
      assert( np.allclose(A, hGold, atol=1e-12) )

    if self.callCount_ == 3:
      bGold = np.ones(3)*(-136.6)
      assert( np.allclose(b, bGold, atol=1e-12) )
      hGold = np.ones((3,3))*22.
      assert( np.allclose(A, hGold, atol=1e-12) )

#----------------------------------------
class OdeObserver:
  def __call__(self, timeStep, time, state):
    print("obs", state)
    assert(state.shape[0]==3)
    print("\n")

#----------------------------
def test():
  '''
  this test checks that the hyp-red lspg problem with bdf2  works as expected.
  Note that at first step, it calls bdf1 and then to do step 2 it calls bdf2.
  We don't solve a real problem but we we use fake data and veryify that
  at every stage involved, from constructor to fom querying to solve,
  the data is supposed to be correct.
  We have the following:
  - stencils mesh: 10 points enumerated as {0,1,...,9}
     o  o  o  o  o  o  o  o  o  o  (points)
     0  1  2  3  4  5  6  7  8  9  (indices of sample mesh points)

  - sample mesh: subset of stencils mesh points {1,4,5,7,8}
        *        *  *     *  *
        0        4  5     7  8    (indices wrt stencil mesh)
        0        1  2     3  4    (enumaration wrt to sample mesh only)

  - romSize = 3
  - start from romState = [0,0,0]
  - use GN with normal equations and do two iterations of the GN solver,
    inner linear solve: simply sets correction to be [1 1 ... 1]
  - linear mapping such that:
              phi = [ 1 1 1;
                      ...  ;
                     10 10 10]

  - we do two time steps: t0 -> t1 -> t2, with dt = 3.0
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
  - done by bdf1 by default because bdf2 needs an auxiliary
    stepper for first iteration

  *************************************
  *** first call to solver we have ***
  *************************************
  romState = [0 0 0],
  y_n      = [0 ... 0]
  y_n-1    = [0 ... 0]

  R = y_n - y_nm-1 - dt*f
    = [-3.3 -6.6 -9.9 -13.2 -16.5]

  lspgJac = I*phi - dt*df/dy*phi
  where I*phi is only taken at the sample mesh points
  and df/dy*phi = [2 2 2;
                   3 3 3;
                   4 4 4;
                   5 5 5;
                   6 6 6]
  we get:
  lspgJac = [2-dt*2 2-dt*2 2-dt*2;    [-4 -4 -4;
             5-dt*3 5-dt*3 5-dt*3;     -4 -4 -4;
             6-dt*4 6-dt*4 6-dt*4; =   -6 -6 -6;
             8-dt*5 8-dt*5 8-dt*5;     -7 -7 -7;
             9-dt*6 9-dt*6 9-dt*6]     -9 -9 -9]

  so that the linear solver should have:
  b = -lspgJac^T R = [ -339.9 -339.9 -339.9 ]
  neg sign because of the sign convention in pressio

  A = (lspgJac)^T (lspgJac) =
        [198 198 198;
        198 198 198;
        198 198 198]

  *************************************
  *** second call to nonlin solver we have ***
  *************************************
  romState  = [1 1 1],
  y_n       = [3 6 9 12 15 18 21 24 27 30]
  y_n-1     = [0 ... 0]

  R = y_n - y_nm-1 - dt*f
   = [6-3.3 15-6.6 18-9.9 24-13.2 27-16.5]

  lspgJac = I*phi - dt*df/dy*phi
  where I*phi is only taken at the sample mesh points
  and df/dy*phi = [2 2 2;
                   3 3 3;
                   4 4 4;
                   5 5 5;
                   6 6 6]
  we get:
  lspgJac = [2-dt*2 2-dt*2 2-dt*2;    [-4 -4 -4;
             5-dt*3 5-dt*3 5-dt*3;     -4 -4 -4;
             6-dt*4 6-dt*4 6-dt*4; =   -6 -6 -6;
             8-dt*5 8-dt*5 8-dt*5;     -7 -7 -7;
             9-dt*6 9-dt*6 9-dt*6]     -9 -9 -9]

  so that the linear solver should have:
  b = -lspgJac^T R = [ 263.1 263.1 263.1 ]
  the neg sign because of the sign convention in pressio

  A = (lspgJac)^T (lspgJac) =
  [198 198 198;
   198 198 198;
   198 198 198]

  =========================
  =========================
        time step 2
  =========================
  =========================
  - done by bdf2

  *************************************
  *** first call to nonlin solver we have ***
  *************************************
  romState = [1 1 1],
  y_n      = [3 6 9 12 15 18 21 24 27 30]
  y_n-1    = [3 6 9 12 15 18 21 24 27 30]
  y_n-2    = [0 ... 0]

  R = y_n - 4/3*y_n-1 + 1/3*yn-2 - 2/3*dt*f
    = y_n - 4/3*y_n-1 - 2*f  (because dt=3)
    = [-4.2 -9.4 -12.6 -16.8 -20]

  lspgJac = I*phi - 2/3*dt*df/dy*phi
  where I*phi is only taken at the sample mesh points
  and df/dy*phi = [2 2 2;
                   3 3 3;
                   4 4 4;
                   5 5 5;
                   6 6 6]
  we get:
  lspgJac= [2-2*2 2-2*2 2-2*2;    [-2 -2 -2;
            5-2*3 5-2*3 5-2*3;     -1 -1 -1;
            6-2*4 6-2*4 6-2*4; =   -2 -2 -2;
            8-2*5 8-2*5 8-2*5;     -2 -2 -2;
            9-2*6 9-2*6 9-2*6]     -3 -3 -3;

  so that the first call to the linear solver should have:
  b = -lspgJac^T R = [ -136.6 -136.6 -136.6]
  neg sign because of the sign convention in pressio

  A = (lspgJac)^T (lspgJac) =
        [22 22 22;
        22 22 22;
        22 22 22]
  '''

  Nstencil = 10
  Nsmesh   = 5
  romSize = 3
  dt = 3.

  appObj  = MyTestApp(Nstencil, Nsmesh, romSize)
  yRef    = np.zeros(Nstencil)
  phi     = np.zeros((Nstencil, romSize), order='F')
  for i in range(Nstencil): phi[i,:] = float(i+1)

  decoder = rom.Decoder(phi)
  yRom    = np.zeros(romSize)

  indexing = rom.lspg.unsteady.StencilToSampleIndexing([1,4,5,7,8])
  scheme = ode.stepscheme.BDF2
  lspgProblem = rom.lspg.unsteady.HypredProblem(scheme, appObj, decoder, yRom, yRef, indexing)
  stepper = lspgProblem.stepper()

  # linear and non linear solver
  lsO = MyLinSolver()
  nlsO = solvers.create_gauss_newton(stepper, yRom, lsO)
  nlsO.setUpdatingCriterion(solvers.update.Standard)
  nlsO.setMaxIterations(2)
  nlsO.setStoppingCriterion(solvers.stop.AfterMaxIters)

  # solve
  myObs = OdeObserver()
  ode.advance_n_steps_and_observe(stepper, yRom, 0., dt, 2, myObs, nlsO)

  fomRecon = lspgProblem.fomStateReconstructor()
  yFomFinal = fomRecon(yRom)
  np.set_printoptions(precision=15)
  # the goldFomState = phi*[2 2 2]
  goldFomState = [6, 12, 18, 24, 30, 36, 42, 48, 54, 60]
  assert( np.allclose(yFomFinal, goldFomState, atol=1e-13) )
