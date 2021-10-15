
import numpy as np
from scipy import linalg
import pathlib, sys
file_path = pathlib.Path(__file__).parent.absolute()

from pressio4py import solvers, ode, rom
from pressio4py.apps.burgers1d import Burgers1d

np.set_printoptions(linewidth=140)

# gold solution expected from this test
gold = np.array([
  1.20961484805614e+00,
  1.00414341203934e+00,
  1.00225747155818e+00,
  1.00247921957863e+00,
  1.00273983077854e+00,
  1.00302857017833e+00,
  1.00334574603388e+00,
  1.00369802443198e+00,
  1.00408787911375e+00,
  1.00451653245805e+00,
  1.00499222980303e+00,
  1.00551742704848e+00,
  1.00609739288540e+00,
  1.00673920319338e+00,
  1.00744706708639e+00,
  1.00823132823391e+00,
  1.00909787535280e+00,
  1.01005373889436e+00,
  1.01111169307645e+00,
  1.01228245079179e+00])

#----------------------------
class MyLinSolver:
  def solve(self, A,b,x):
    lumat, piv, info = linalg.lapack.dgetrf(A, overwrite_a=True)
    # here we must use x[:] otherwise it won't overwrite x passed in
    x[:], info = linalg.lapack.dgetrs(lumat, piv, b, 0, 0)

#----------------------------------------
class OdeObserver:
  def __init__(self): pass

  def __call__(self, timeStep, time, state):
    print(state)
    assert(state.shape[0]==11)

#----------------------------
def test_euler():
  meshSize = 20
  romSize  = 11
  Nsteps   = 10
  dt       = 0.01
  t0       = 0.

  # create app
  appObj = Burgers1d(meshSize)

  # set reference state
  yRef = np.ones(meshSize)

  # I have to make phi a column-major array to ensure
  # pressio does not make a copy of this
  basisFile = str(file_path) + "/basis_euler.txt"
  phi = np.copy(np.loadtxt(basisFile), order='F')

  # decoder
  decoder = rom.Decoder(phi)
  # LSPG state
  yRom = np.zeros(romSize)

  scheme = ode.stepscheme.BDF1
  problem = rom.lspg.unsteady.DefaultProblem(scheme, appObj, decoder, yRom, yRef)

  # linear and non linear solver
  lsO = MyLinSolver()
  nlsO = solvers.create_levenberg_marquardt(problem, yRom, lsO)
  nlsO.setUpdatingCriterion(solvers.update.LMSchedule1)
  nlsO.setMaxIterations(2)

  # solve
  myObs = OdeObserver()
  ode.advance_n_steps_and_observe(problem, yRom, t0,dt, Nsteps, myObs, nlsO)

  fomRecon = problem.fomStateReconstructor()
  yFomFinal = fomRecon(yRom)
  print(yFomFinal)

  for y1,y2 in zip(gold, yFomFinal):
    assert( np.abs(y1-y2) < 1e-10)


if __name__ == "__main__":
  test_euler()
