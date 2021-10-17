
import numpy as np
from pressio4py import solvers, ode, rom
from pressio4py.rom import galerkin, lspg

class TrivialFomVeloOnly:
  def __init__(self, N):
    self.N_ = N

  def createVelocity(self):
    return np.zeros(self.N_)

  def velocity(self, stateIn, time, f):
    f[:] = np.random.rand(self.N_)

class TrivialFomVeloAndJac:
  def __init__(self, N):
    self.N_ = N

  def createVelocity(self):
    return np.zeros(self.N_)

  def createApplyJacobianResult(self, B):
    return np.zeros((self.N_, B.shape[1]))

  def velocity(self, stateIn, time, R):
    R[:] = stateIn[:] + time

  def applyJacobian(self, u, B, t, A):
    A[:,:] = np.random.rand(self.N_, B.shape[1])

class MyLinSolver:
  def solve(self, A,b,x):
    x[:] += 1.

class MyObs:
  def __call__(self, step, time, state):
    print(state)

def test_snip1():
  print("\n")
  print("snip1")
  N = 10
  fomObj = TrivialFomVeloOnly(N)
  fomState  = np.zeros(N)
  phi       = np.ones((N,3), order='F')
  decoder   = rom.Decoder(phi)
  romState  = np.array([0., 1., 2.])
  scheme    = ode.stepscheme.RungeKutta4
  problem   = galerkin.DefaultExplicitProblem(scheme, fomObj, decoder, romState, fomState)

  dt = 0.5
  problem(romState, 0., dt, 1)
  print(romState)

  romState  = np.array([0., 1., 2.])
  nSteps = 1
  obs = MyObs()
  ode.advance_n_steps_and_observe(problem, romState, 0., dt, nSteps, obs)

def test_snip2():
  print("\n")
  print("snip2")
  N = 10
  fomObj    = TrivialFomVeloAndJac(N)
  fomState  = np.zeros(N)
  phi       = np.ones((N,3), order='F')
  decoder   = rom.Decoder(phi)
  romState  = np.array([0., 1., 2.])
  scheme    = ode.stepscheme.BDF1
  problem   = galerkin.DefaultImplicitProblem(scheme, fomObj, decoder, romState, fomState)

  lsO = MyLinSolver()
  nlsO = solvers.create_newton_raphson(problem, romState, lsO)
  nlsO.setUpdatingCriterion(solvers.update.Standard)
  nlsO.setMaxIterations(5)
  nlsO.setStoppingCriterion(solvers.stop.AfterMaxIters)

  time, dt = 0., 0.5
  for i in range(5):
    problem(romState, time, dt, i, nlsO)
    print(romState)

  obs = MyObs()
  ode.advance_n_steps_and_observe(problem, romState, 0., 0.1, 2, obs, nlsO)





# class MyDtSetter:
#   def __call__(self, step, time):
#     print("MyDtSetter")
#     return 1.

# def test_galerkin_cont_time_default_explicit2():
#   print("\n")
#   print("call advance_n_steps custom dt")
#   N = 10
#   sw = MySys(N)
#   fom_state = np.zeros(N)
#   phi = np.zeros((N,3), order='F')
#   phi[:,0] = 0.
#   phi[:,1] = 1.
#   phi[:,2] = 2.
#   decoder = rom.Decoder(phi)
#   rom_state = np.array([0., 1., 2.])
#   scheme = ode.stepscheme.ForwardEuler
#   problem = rom.galerkin.DefaultExplicitProblem(scheme, sw, decoder, rom_state, fom_state)
#   dtCb = MyDtSetter()
#   num_steps = 2
#   obs = MyObs()
#   ode.advance_n_steps_and_observe(problem, rom_state, 0., dtCb, num_steps, obs)
#   print(rom_state)
#   assert(rom_state[0] == 0.)
#   assert(rom_state[1] == 2611.)
#   assert(rom_state[2] == 5222.)

# def test_galerkin_cont_time_default_explicit3():
#   print("\n")
#   print("using call operator")
#   N = 10
#   sw = MySys(N)
#   fom_state = np.zeros(N)
#   phi = np.zeros((N,3), order='F')
#   phi[:,0] = 0.
#   phi[:,1] = 1.
#   phi[:,2] = 2.
#   decoder = rom.Decoder(phi)
#   rom_state = np.array([0., 1., 2.])
#   scheme = ode.stepscheme.ForwardEuler
#   problem = rom.galerkin.DefaultExplicitProblem(scheme, sw, decoder, rom_state, fom_state)
#   dt = 1.
#   problem(rom_state, 0., dt, 1)
#   print(rom_state)
#   assert(rom_state[0] == 0.)
#   assert(rom_state[1] == 51.)
#   assert(rom_state[2] == 102.)
