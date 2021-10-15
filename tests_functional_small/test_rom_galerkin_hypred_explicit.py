
import numpy as np
from pressio4py import ode, rom

class MySys:
  def __init__(self, N):
    self.N_ = N
    self.indices_ = np.array([1,3,5,7,9,11,13,15,17,19])

  def createVelocity(self):
    return np.zeros(self.N_)

  def velocity(self, stateIn, time, R):
    for c,i in enumerate(self.indices_):
      R[c] = stateIn[i] + time

class MyProjector:
  def __init__(self, phi):
    self.phi_ = phi

  def __call__(self, operand, time, result):
    result[:] = np.dot(self.phi_.T, operand)

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

def test_galerkin_cont_time_hypred_explicit():
  nstencil = 20
  nsample = 10
  sw = MySys(nsample)
  fom_state = np.zeros(nstencil)

  phi = np.zeros((nstencil,3), order='F')
  phi[:,0] = 0.
  phi[:,1] = 1.
  phi[:,2] = 2.
  phi[0, :] = -111.
  phi[2, :] = -111.
  phi[4, :] = 111.
  phi[6, :] = 423.
  phi[8, :] = -21.
  phi[10, :] = 423.
  phi[12, :] = -21.
  phi[14, :] = 423.
  phi[16, :] = -21.
  phi[18, :] = -21.

  decoder = rom.Decoder(phi)
  rom_state = np.array([0., 1., 2.])

  phiForProj = np.zeros((nsample,3), order='F')
  phiForProj[:,0] = 0.
  phiForProj[:,1] = 1.
  phiForProj[:,2] = 2.
  projector = MyProjector(phiForProj)

  scheme = ode.stepscheme.ForwardEuler
  problem = rom.galerkin.HyperreducedExplicitProblem(scheme, sw, decoder, rom_state, \
                                                     fom_state, projector)

  dt = 1.
  num_steps = 2
  obs = MyObs()
  ode.advance_n_steps_and_observe(problem, rom_state, 0., dt, num_steps, obs)
  print(rom_state)
  assert(rom_state[0] == 0.)
  assert(rom_state[1] == 2611.)
  assert(rom_state[2] == 5222.)
