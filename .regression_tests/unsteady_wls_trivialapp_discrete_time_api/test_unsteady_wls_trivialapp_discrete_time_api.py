
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
class MyTestApp:
  def __init__(self, N):
    self.N_ = N
    self.callCount_ = 0

  def createDiscreteTimeResidual(self):
    return np.zeros(self.N_)

  def createApplyDiscreteTimeJacobianResult(self, B):
    return np.zeros((self.N_, B.shape[1]))

  def discreteTimeResidual(self, step, time, dt, R, ynp1, yn):
    self.callCount_ += 1
    assert(len(R) == self.N_)
    assert(len(ynp1) == self.N_)
    assert(len(yn) == self.N_)

    R[:] = 1. 
    # print("ynp1")
    # print(ynp1)
    # print("yn")
    # print(yn)

  def applyDiscreteTimeJacobian(self, step, time, dt, B, A, ynp1, yn):
    A[0,:] = 0.
    A[1,:] = 2.
    A[2,:] = 1.
    A[3,:] = 3.
    A[4,:] = 2.
    A[5,:] = 4.
    A[6,:] = 5.


#----------------------------
class MyLinSolver:
  def __init__(self): pass

  def solve(self, A,b,x):
    print("My lin solver")
    # # here we use 1 because for wls, A is lower-triangular
    # useLower = 1
    # C, info0 = linalg.lapack.dpotrf(A, useLower, 0)
    # assert(info0==0)
    # x[:], info1 = linalg.lapack.dpotrs(C, b, useLower, 0)
    # assert(info0==0)
    x[:] = 1.

def test1():
  logger.initialize(logger.logto.terminal, "null")
  logger.setVerbosity([logger.loglevel.debug])

  meshSize = 20
  romSize  = 11
  Nsteps   = 10
  dt       = 0.01
  t0       = 0.

  # create app
  appObj = MyTestApp(meshSize)

  yRef = np.ones(meshSize)
  phi = np.ones((meshSize, romSize),order='F')
  decoder = rom.Decoder(phi)

  numStepsInWindow = 1
  wlsSize = romSize*numStepsInWindow
  finalTime = 0.1
  numWindows = int((finalTime/dt)/numStepsInWindow)
  print("numWindows = {}".format(numWindows))

  # WLS state
  wlsState = np.zeros(wlsSize)

  wlsPolicy = rom.exp.wls.default.SequentialPolicyDiscreteTimeBDF1(romSize, numStepsInWindow, decoder, appObj, yRef)
  wlsSystem = rom.exp.wls.default.ProblemDiscreteTimeBDF1(decoder, wlsPolicy, yRef, yRef, wlsState)

  nonLinSolver = solvers.createGaussNewton(wlsSystem, wlsState, MyLinSolver())
  nonLinSolver.setMaxIterations(1)

  rom.exp.wls.solveWindowsSequentially(wlsSystem, wlsState, nonLinSolver, numWindows, dt)

  # startI    = (numStepsInWindow-1)*romSize
  # wlsSpan   = wlsState[startI:startI+romSize]
  # fomRecon  = wlsSystem.fomStateReconstructor()
  # yFomFinal = fomRecon.evaluate(wlsState)
  # print(wlsState)
  # gold = np.loadtxt(str(file_path)+"/goldrom.txt")
  # assert( np.allclose(gold, wlsState, atol=1e-14) )

#----------------------------
if __name__ == "__main__":
  test1()
