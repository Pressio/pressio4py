
import numpy as np
from pressio4py import ode, rom

class MySys:
  def __init__(self, N, indices_to_corrupt):
    self.N_ = N
    self.indices_to_corrupt_ =indices_to_corrupt

  def createVelocity(self):
    return np.zeros(self.N_)

  def velocity(self, stateIn, time, R):
    R[:] = stateIn[:] + time
    for i in self.indices_to_corrupt_:
      R[i] = -1114.

class MyMasker:
  def __init__(self, sample_indices):
    self.sample_indices_ = sample_indices

  def createApplyMaskResult(self, operand):
    return np.zeros(len(self.sample_indices_))

  def __call__(self, operand, time, result):
    result[:] = np.take(operand, self.sample_indices_)

class MyProjector:
  def __init__(self, phi):
    self.phi_ = phi

  def __call__(self, operand, time, result):
    result[:] = np.dot(self.phi_.T, operand)


class MyObs:
  def __call__(self, step, time, state):
    print(state)

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

def test_galerkin_cont_time_masked_explicit():
  N = 20

  sample_indices  = np.array([0,2,4,6,8,10,12,14,16,18])
  Nmasked = len(sample_indices)
  corrupt_indices = np.array([1,7,13,19])

  sw = MySys(N, corrupt_indices)
  fom_state = np.zeros(N)
  phi = np.zeros((N,3), order='F')
  phi[:,0] = 0.
  phi[:,1] = 1.
  phi[:,2] = 2.
  # corrupting the entries in phiFull is only way
  # we can ensure the masking works properly
  phi[1, :] = -111.
  phi[7, :] = 111.
  phi[11, :] = 423.
  phi[17, :] = -21.

  decoder = rom.Decoder(phi)
  rom_state = np.array([0., 1., 2.])

  masker = MyMasker(sample_indices)
  phiSample = np.zeros((Nmasked,3), order='F')
  phiSample[:,0] = 0.
  phiSample[:,1] = 1.
  phiSample[:,2] = 2.
  projector = MyProjector(phiSample)

  scheme = ode.stepscheme.ForwardEuler
  problem = rom.galerkin.MaskedExplicitProblem(scheme, sw, decoder, rom_state, \
                                               fom_state, projector, masker)

  dt = 1.
  num_steps = 2
  obs = MyObs()
  ode.advance_n_steps_and_observe(problem, rom_state, 0., dt, num_steps, obs)
  print(rom_state)
  assert(rom_state[0] == 0.)
  assert(rom_state[1] == 2611.)
  assert(rom_state[2] == 5222.)
