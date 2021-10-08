
import numpy as np
from scipy import linalg
import pathlib, sys, os
file_path = pathlib.Path(__file__).parent.absolute()

from pressio4py import logger as logger
from pressio4py import rom as rom
from pressio4py import solvers as solvers
from pressio4py.apps.burgers1d import Burgers1d

np.set_printoptions(linewidth=240)

#----------------------------
class MyLinSolver:
  def __init__(self): pass

  def solve(self, A,b,x):
    print("My lin solver")
    # here we use 1 because for wls, A is lower-triangular
    useLower = 1
    C, info0 = linalg.lapack.dpotrf(A, useLower, 0)
    assert(info0==0)
    x[:], info1 = linalg.lapack.dpotrs(C, b, useLower, 0)
    assert(info0==0)

def test1():
  logger.initialize(logger.logto.terminal, "null")
  logger.setVerbosity([logger.loglevel.debug])

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
  basisFile = str(file_path) + "/basis.txt"
  phi = np.copy(np.loadtxt(basisFile), order='F')

  # decoder
  decoder = rom.Decoder(phi)

  numStepsInWindow = 5
  wlsSize = romSize*numStepsInWindow
  finalTime = 0.1
  numWindows = int((finalTime/dt)/numStepsInWindow)
  print("numWindows = {}".format(numWindows))

  # WLS state
  wlsState = np.zeros(wlsSize)

  wlsPolicy = rom.exp.wls.default.SequentialPolicyBDF1(romSize, numStepsInWindow, decoder, appObj, yRef)
  wlsSystem = rom.exp.wls.default.ProblemBDF1(decoder, wlsPolicy, yRef, yRef, wlsState)

  nonLinSolver = solvers.createGaussNewton(wlsSystem, wlsState, MyLinSolver())
  nonLinSolver.setMaxIterations(2)

  rom.exp.wls.solveWindowsSequentially(wlsSystem, wlsState, nonLinSolver, numWindows, dt)

  startI    = (numStepsInWindow-1)*romSize
  wlsSpan   = wlsState[startI:startI+romSize]
  fomRecon  = wlsSystem.fomStateReconstructor()
  yFomFinal = fomRecon.evaluate(wlsState)

  print(wlsState)

  gold = np.loadtxt(str(file_path)+"/goldrom.txt")
  assert( np.allclose(gold, wlsState, atol=1e-14) )

#----------------------------
if __name__ == "__main__":
  test1()
