
import numpy as np
from pressio4py import solvers, ode, rom
from pressio4py.rom import galerkin


class MySys:
  def __init__(self, N):
    self.N_ = N

  def createVelocity(self):
    return np.zeros(self.N_)

  def velocity(self, stateIn, time, R):
    R[:] = stateIn[:] + time

class MyObs:
  def __call__(self, step, time, state):
    if (step ==0):
      assert( state[0] == 0.)
      assert( state[1] == 1.)
      assert( state[2] == 2.)

    if (step ==1):
      assert( state[0] == 0.)
      assert( state[1] == 51.)
      assert( state[2] == 102.)

    if (step == 2):
      assert( state[0] == 0.)
      assert( state[1] == 2611.)
      assert( state[2] == 5222.)

def test_galerkin_cont_time_default_explicit1():
  print("\n")
  print("call advance_n_steps")
  N = 10
  sw = MySys(N)
  fom_state = np.zeros(N)
  phi = np.zeros((N,3), order='F')
  phi[:,0] = 0.
  phi[:,1] = 1.
  phi[:,2] = 2.
  decoder = rom.Decoder(phi)
  rom_state = np.array([0., 1., 2.])
  scheme = ode.stepscheme.ForwardEuler
  problem = galerkin.DefaultExplicitProblem(scheme, sw, decoder, rom_state, fom_state)
  stepper = problem.stepper()
  dt = 1.
  num_steps = 2
  obs = MyObs()
  ode.advance_n_steps_and_observe(stepper, rom_state, 0., dt, num_steps, obs)
  print(rom_state)
  assert(rom_state[0] == 0.)
  assert(rom_state[1] == 2611.)
  assert(rom_state[2] == 5222.)

class MyDtSetter:
  def __call__(self, step, time):
    print("MyDtSetter")
    return 1.

def test_galerkin_cont_time_default_explicit2():
  print("\n")
  print("call advance_n_steps custom dt")
  N = 10
  sw = MySys(N)
  fom_state = np.zeros(N)
  phi = np.zeros((N,3), order='F')
  phi[:,0] = 0.
  phi[:,1] = 1.
  phi[:,2] = 2.
  decoder = rom.Decoder(phi)
  rom_state = np.array([0., 1., 2.])
  scheme = ode.stepscheme.ForwardEuler
  problem = rom.galerkin.DefaultExplicitProblem(scheme, sw, decoder, rom_state, fom_state)
  stepper = problem.stepper()
  dtCb = MyDtSetter()
  num_steps = 2
  obs = MyObs()
  ode.advance_n_steps_and_observe(stepper, rom_state, 0., dtCb, num_steps, obs)
  print(rom_state)
  assert(rom_state[0] == 0.)
  assert(rom_state[1] == 2611.)
  assert(rom_state[2] == 5222.)

def test_galerkin_cont_time_default_explicit3():
  print("\n")
  print("using call operator")
  N = 10
  sw = MySys(N)
  fom_state = np.zeros(N)
  phi = np.zeros((N,3), order='F')
  phi[:,0] = 0.
  phi[:,1] = 1.
  phi[:,2] = 2.
  decoder = rom.Decoder(phi)
  rom_state = np.array([0., 1., 2.])
  scheme = ode.stepscheme.ForwardEuler
  problem = rom.galerkin.DefaultExplicitProblem(scheme, sw, decoder, rom_state, fom_state)
  stepper = problem.stepper()
  dt = 1.
  stepper(rom_state, 0., dt, 1)
  print(rom_state)
  assert(rom_state[0] == 0.)
  assert(rom_state[1] == 51.)
  assert(rom_state[2] == 102.)
