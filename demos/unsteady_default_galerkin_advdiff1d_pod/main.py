
import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt

# if run from within a build of pressio4py, need to append to python path
import pathlib, sys
file_path = pathlib.Path(__file__).parent.absolute()
sys.path.append(str(file_path) + "/../../apps") # to access the apps
sys.path.append(str(file_path) + "/../..")      # to access pressio4py lib
sys.path.append(str(file_path) + "/..")         # to access fom

from adv_diff1d import *
from pressio4py import rom as rom
from pressio4py import solvers as solvers
from adv_diff_1d_fom import doFom

#----------------------------------------
def computePodModes(snapshots):
  print("SVD on matrix: ", snapshots.shape)
  U,S,VT = np.linalg.svd(snapshots)
  return U

#----------------------------------------
def runGalerkin(fomObj, dt, nsteps, modes):
  # auxiliary class to use in the solve below
  # to monitor the rom state during time stepping
  class RomStateObserver:
    def __init__(self): pass
    def __call__(self, timeStep, time, state): pass

  # find out number of modes wanted
  romSize = modes.shape[1]

  # create a linear decoder, passing only the desired number of modes
  # this will make a deep copy of the modes
  linearDecoder = rom.Decoder(modes)

  # fom reference state: here it is zero
  fomReferenceState = np.zeros(fomObj.nGrid)

  # create ROM state by projecting the fom initial condition
  fomInitialState = fomObj.u0.copy()
  romState = np.dot(modes.T, fomInitialState)

  # create GALERKIN problem
  problem = rom.galerkin.default.ProblemEuler(fomObj, linearDecoder, romState, fomReferenceState)

  # create object to monitor the romState at every iteration
  myObs = RomStateObserver()
  # solver GALERKIN problems
  rom.galerkin.advanceNSteps(problem, romState, 0., dt, nsteps, myObs)

  # after we are done, use the reconstructor object to reconstruct the fom state
  # get the reconstructor object: this allows to map romState to fomState
  fomRecon = problem.fomStateReconstructor()
  return fomRecon.evaluate(romState)

######## MAIN ###########
if __name__ == "__main__":
  # initial condition u(x,t=0)
  ic = lambda x: 2.*np.sin(9.*np.pi*x) - np.sin(4.*np.pi*x)
  # create fom object
  fomObj = AdvDiff1d(nGrid=120, IC=ic, adv_coef=2.0)

  # the final time to integrate to
  finalTime = .05

  #--- 1. FOM ---#
  fomTimeStepSize  = 1e-5
  fomNumberOfSteps = int(finalTime/fomTimeStepSize)
  sampleEvery      = 200
  [fomFinalState, snapshots] = doFom(fomObj, fomTimeStepSize, fomNumberOfSteps, sampleEvery)

  #--- 2. POD ---#
  modes = computePodModes(snapshots)

  #--- 3. GALERKIN ROM ---#
  romSize = 5
  romTimeStepSize  = 3e-4
  romNumberOfSteps = int(finalTime/romTimeStepSize)
  # we pass only romSize modes
  approximatedState = runGalerkin(fomObj, romTimeStepSize,
                              romNumberOfSteps, modes[:,:romSize])

  # compute l2-error between fom and approximate state
  fomNorm = linalg.norm(fomFinalState)
  err = linalg.norm(fomFinalState-approximatedState)
  print("Final state relative l2 error: {}".format(err/fomNorm))

  #--- plot ---#
  ax = plt.gca()
  ax.plot(fomObj.xGrid, fomFinalState, '-', linewidth=2, label='FOM')
  ax.plot(fomObj.xGrid, approximatedState, 'or', label='Galerkin: '+str(romSize)+' POD modes')
  plt.rcParams.update({'font.size': 18})
  plt.ylabel("Solution", fontsize=18)
  plt.xlabel("x-coordinate", fontsize=18)
  plt.legend(fontsize=12)
  plt.show()
