
import numpy as np
from pressio4py import solvers, ode, rom

class FomSystem:
  def __init__(self, N):
    self.N_ = N

  def createVelocity(self):
    return np.zeros(self.N_)

  def createApplyJacobianResult(self, B):
    return np.zeros((self.N_, B.shape[1]))

  def velocity(self, stateIn, time, R):
    R[:] = stateIn[:] + time

  def applyJacobian(self, u, B, t, A):
    # this computes: A = J*B
    # where J is supposed to be the fom jacobian
    assert(A.shape[0] == self.N_)
    A[:,:] = np.copy(B)
    A[:,:] += t

class MyNonLinSolver:
  def __init__(self, system):
    self.R = system.createResidual()
    self.J = system.createJacobian()
    self.call_count = 0

  def solve(self, system, x):
    print("Python NonLin solver")
    self.call_count += 1

    system.residual(x, self.R);
    system.jacobian(x, self.J);
    print(self.R)
    print(self.J)

def test_lspg_cont_time_default_2():
  print("\n")
  print("using advance")

  N = 8
  sw = FomSystem(N)
  fom_state = np.zeros(N)
  phi = np.zeros((N,3), order='F')
  count = 0
  for i in range(N):
    for j in range(3):
      phi[i,j] = float(count)
      count += 1

  decoder = rom.Decoder(phi)
  rom_state = np.array([0., 1., 2.])
  scheme = ode.stepscheme.BDF1
  problem = rom.lspg.unsteady.DefaultProblem(scheme, sw, decoder, rom_state, fom_state)

  mynlsO = MyNonLinSolver(problem)
  dt, num_steps = 2., 1
  ode.advance_n_steps(problem, rom_state, 0., dt, num_steps, mynlsO)
  print("final rom_state = ", rom_state)
