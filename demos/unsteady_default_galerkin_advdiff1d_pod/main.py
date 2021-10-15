
import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt

# if run from within a build of pressio4py, need to append to python path
import pathlib, sys
file_path = pathlib.Path(__file__).parent.absolute()
sys.path.append(str(file_path) + "/..")         # to access doFom

from pressio4py import logger, solvers, ode, rom
from pressio4py.apps.advection_diffusion1d import AdvDiff1d
from adv_diff_1d_fom import doFom
from settings_for_website import edit_figure_for_web

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

  # create problem
  scheme = ode.stepscheme.ForwardEuler
  problem = rom.galerkin.DefaultExplicitProblem(scheme, fomObj, linearDecoder, romState, fomReferenceState)

  # create object to monitor the romState at every iteration
  myObs = RomStateObserver()
  # solve problem
  ode.advance_n_steps_and_observe(problem, romState, 0., dt, nsteps, myObs)

  # after we are done, use the reconstructor object to reconstruct the fom state
  # get the reconstructor object: this allows to map romState to fomState
  fomRecon = problem.fomStateReconstructor()
  return fomRecon(romState)

######## MAIN ###########
if __name__ == "__main__":
  logger.initialize(logger.logto.terminal)
  logger.setVerbosity([logger.loglevel.info])

  # create fom object
  fomObj = AdvDiff1d(nGrid=120, adv_coef=2.0)

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
  romTimeStepSize  = 3e-4
  romNumberOfSteps = int(finalTime/romTimeStepSize)
  # run with various number of modes
  romSizes = [2,4,6]
  approximations = {}
  for romSize in romSizes:
    currentSolution = runGalerkin(fomObj, romTimeStepSize,
                                  romNumberOfSteps,
                                  modes[:,:romSize])
    approximations[romSize] = currentSolution

    # compute l2-error between fom and approximate state
    fomNorm = linalg.norm(fomFinalState)
    err = linalg.norm(fomFinalState-currentSolution)
    print("With {} modes, final relative l2 error: {}".format(romSize, err/fomNorm))

  logger.finalize()

  #--- plot ---#
  ax = plt.gca()
  plt.rcParams.update({'font.size': 18})
  ax.plot(fomObj.xGrid, fomFinalState, '-g', linewidth=2, label='FOM')

  colors = ['b', 'r', 'y']
  for [k,v],c in zip(approximations.items(), colors):
    ax.plot(fomObj.xGrid, v, 'o', markeredgecolor=c,
            markerfacecolor='None', markersize=3,
            label='Galerkin: '+str(k)+' POD modes')

  ax.set_ylabel("Solution", fontsize=18)
  ax.set_xlabel("x-coordinate", fontsize=18)
  leg = plt.legend(fontsize=12, fancybox=True, framealpha=0, loc='lower right')
  ax.grid(True, linewidth=0.35, color='gray')

  #used to change color to text and axes
  edit_figure_for_web(ax, leg)
  plt.savefig('demo1.png', dpi=250, transparent=True)
  plt.show()
