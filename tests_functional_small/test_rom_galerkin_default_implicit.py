
import numpy as np
from pressio4py import solvers, ode, rom

class FomSystem:
  def __init__(self, N):
    self.N_ = N

  def createVelocity(self):
    print("sys2: createVelocity")
    return np.zeros(self.N_)

  def createApplyJacobianResult(self, B):
    print("sys2: createApplyJacobianResult")
    return np.zeros((self.N_, B.shape[1]))

  def velocity(self, stateIn, time, R):
    print("sys2: velocity")
    R[:] = stateIn[:] + time

  def applyJacobian(self, u, B, t, A):
    # this computes: A = J*B
    # where J is supposed to be the fom jacobian
    assert(A.shape[0] == self.N_)
    A[:,:] = np.copy(B)
    A[:,:] += t

class MyLinSolver:
  def solve(self, A,b,x):
    print("Python Lin solver")
    print(A)
    print(b)
    print(x)
    x[:] += 1.

class MyObs2:
  def __call__(self, step, time, state):
    print("MyObs2")
    print(state)

# def test_galerkin_cont_time_default_implicit1():
#   print("\n")
#   N = 10
#   sw = FomSystem(N)
#   fom_state = np.zeros(N)
#   phi = np.zeros((N,3), order='F')
#   phi[:,0] = 0.
#   phi[:,1] = 1.
#   phi[:,2] = 2.
#   decoder = rom.Decoder(phi)
#   rom_state = np.array([0., 1., 2.])
#   scheme = ode.stepscheme.BDF1
#   problem = rom.galerkin.DefaultImplicitProblem(scheme, sw, decoder, rom_state, fom_state)

#   # solvers
#   lsO = MyLinSolver()
#   nlsO = solvers.create_newton_raphson(problem, rom_state, lsO)
#   nlsO.setUpdatingCriterion(solvers.update.Standard)
#   nlsO.setMaxIterations(2)
#   nlsO.setStoppingCriterion(solvers.stop.AfterMaxIters)

#   dt, num_steps = 1., 1
#   obs = MyObs2()
#   ode.advance_n_steps_and_observe(problem, rom_state, 0., dt, num_steps, obs, nlsO)
#   print("final rom_state = ", rom_state)

# =====================================================================================

class MyNonLinSolver:
  def __init__(self, system):
    self.R = system.createResidual()
    self.J = system.createJacobian()
    self.call_count = 0

  def solve(self, system, x):
    print("Python NonLin solver")
    self.call_count += 1

    if self.call_count==1:
      system.residual(x, self.R);
      system.jacobian(x, self.J);
      assert(self.R[0] == 0.)
      assert(self.R[1] == -140.)
      assert(self.R[2] == -280.)
      assert(self.J[0,0] ==   1.)
      assert(self.J[0,1] ==   0.)
      assert(self.J[0,2] ==   0.)
      assert(self.J[1,0] == -40.)
      assert(self.J[1,1] == -59.)
      assert(self.J[1,2] == -80.)
      assert(self.J[2,0] == -80.)
      assert(self.J[2,1] ==-120.)
      assert(self.J[2,2] ==-159.)

      # fake an update to solution
      x[:] += 1.

      # do another fake iteration
      system.residual(x, self.R);
      system.jacobian(x, self.J);
      assert(self.R[0] == 1.)
      assert(self.R[1] == -199.)
      assert(self.R[2] == -399.)
      assert(self.J[0,0] ==   1.)
      assert(self.J[0,1] ==   0.)
      assert(self.J[0,2] ==   0.)
      assert(self.J[1,0] == -40.)
      assert(self.J[1,1] == -59.)
      assert(self.J[1,2] == -80.)
      assert(self.J[2,0] == -80.)
      assert(self.J[2,1] ==-120.)
      assert(self.J[2,2] ==-159.)

def test_galerkin_cont_time_default_implicit2():
  print("\n")
  print("using advance")

  N = 10
  sw = FomSystem(N)
  fom_state = np.zeros(N)
  phi = np.zeros((N,3), order='F')
  phi[:,0] = 0.
  phi[:,1] = 1.
  phi[:,2] = 2.
  decoder = rom.Decoder(phi)
  rom_state = np.array([0., 1., 2.])
  scheme = ode.stepscheme.BDF1
  problem = rom.galerkin.DefaultImplicitProblem(scheme, sw, decoder, rom_state, fom_state)

  mynlsO = MyNonLinSolver(problem)

  dt, num_steps = 2., 2
  obs = MyObs2()
  ode.advance_n_steps_and_observe(problem, rom_state, 0., dt, num_steps, obs, mynlsO)
  print("final rom_state = ", rom_state)


def test_galerkin_cont_time_default_implicit3():
  print("\n")
  print("using call operator")
  N = 10
  sw = FomSystem(N)
  fom_state = np.zeros(N)
  phi = np.zeros((N,3), order='F')
  phi[:,0] = 0.
  phi[:,1] = 1.
  phi[:,2] = 2.
  decoder = rom.Decoder(phi)
  rom_state = np.array([0., 1., 2.])
  scheme = ode.stepscheme.BDF1
  problem = rom.galerkin.DefaultImplicitProblem(scheme, sw, decoder, rom_state, fom_state)

  mynlsO = MyNonLinSolver(problem)
  problem(rom_state, 0., 2., 1, mynlsO)
  print("final rom_state = ", rom_state)
