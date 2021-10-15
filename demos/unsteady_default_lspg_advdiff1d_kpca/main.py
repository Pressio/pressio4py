
import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt
import sklearn.decomposition as skd

# if run from within a build of pressio4py, need to append to python path
import pathlib, sys
file_path = pathlib.Path(__file__).parent.absolute()
sys.path.append(str(file_path) + "/..")         # to access doFom

from pressio4py import logger, solvers, ode, rom
from pressio4py.apps.advection_diffusion1d import AdvDiff1d
from adv_diff_1d_fom import doFom
from settings_for_website import edit_figure_for_web

class MyMapperKPCA:
  def __init__(self, snapshots, numModes):
    self.transformer_ = skd.KernelPCA(n_components=numModes,\
                                      kernel='poly',
                                      degree=3,
                                      fit_inverse_transform=True)
    # do training using provided snapshots
    self.transformer_.fit(snapshots)

    self.numModes_ = numModes
    fomSize = snapshots.shape[1]
    self.fomState0 = np.zeros(fomSize)
    self.fomState1 = np.zeros(fomSize)
    # attention: the jacobian of the mapping must be column-major oder
    # so that pressio can view it without deep copying it, this enables
    # to keep only one jacobian object around and to call the update
    # method below correctly
    self.jacobian_ = np.zeros((fomSize,numModes), order='F')

  def numModes(self): return self.numModes_

  def jacobian(self): return self.jacobian_

  def applyMapping(self, romState, fomState):
    fomState[:] = np.squeeze(self.transformer_.inverse_transform(romState.reshape(1,-1)))

  def applyInverseMapping(self, romState, fomState):
    romState[:] = np.squeeze(self.transformer_.transform(fomState.reshape(1,-1)))

  def updateJacobian(self, romState):
    romStateLocal = romState.copy()
    # finite difference to approximate jacobian of the mapping
    self.applyMapping(romStateLocal,self.fomState0)
    eps = 0.001
    for i in range(self.numModes_):
        romStateLocal[i] += eps
        self.applyMapping(romStateLocal, self.fomState1)
        self.jacobian_[:,i] = (self.fomState1 - self.fomState0) / eps
        romStateLocal[i] -= eps

#----------------------------------------
def runLspg(fomObj, dt, nsteps, customMapper):
  # this is an auxiliary class that can be passed to solve
  # LSPG to monitor the rom state.
  class RomStateObserver:
    def __call__(self, timeStep, time, state): pass

  # this linear solver is used at each gauss-newton iteration
  class MyLinSolver:
    def solve(self, A,b,x):
      lumat, piv, info = linalg.lapack.dgetrf(A, overwrite_a=True)
      x[:], info = linalg.lapack.dgetrs(lumat, piv, b, 0, 0)

  #----------------------------------------
  # create a custom decoder using the mapper passed as argument
  customDecoder = rom.Decoder(customMapper, "kPCAMapper")

  # fom reference state: here it is zero
  fomReferenceState = np.zeros(fomObj.nGrid)

  # create ROM state by projecting the fom initial condition
  fomInitialState = fomObj.u0.copy()
  romState = np.zeros(customMapper.numModes())
  customMapper.applyInverseMapping(romState, fomInitialState)

  # create LSPG problem
  scheme = ode.stepscheme.BDF1
  problem = rom.lspg.unsteady.DefaultProblem(scheme, fomObj, customDecoder, romState, fomReferenceState)

  # create the Gauss-Newton solver
  nonLinSolver = solvers.create_gauss_newton(problem, romState, MyLinSolver())
  # set tolerance and convergence criteria
  nlsTol, nlsMaxIt = 1e-7, 10
  nonLinSolver.setMaxIterations(nlsMaxIt)
  nonLinSolver.setStoppingCriterion(solvers.stop.WhenCorrectionAbsoluteNormBelowTolerance)

  # create object to monitor the romState at every iteration
  myObs = RomStateObserver()
  # solve problem
  ode.advance_n_steps_and_observe(problem, romState, 0., dt, nsteps, myObs, nonLinSolver)

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

  #--- 2. train a nonlinear mapping using kPCA ---#
  # here we use 3 modes, change this to try different modes
  myNonLinearMapper = MyMapperKPCA(snapshots.T, numModes=3)

  #--- 3. LSPG ROM ---#
  romTimeStepSize  = 3e-4
  romNumberOfSteps = int(finalTime/romTimeStepSize)
  approximatedState = runLspg(fomObj, romTimeStepSize, romNumberOfSteps, myNonLinearMapper)

  # compute l2-error between fom and approximate state
  fomNorm = linalg.norm(fomFinalState)
  err = linalg.norm(fomFinalState-approximatedState)
  print("Final state relative l2 error: {}".format(err/fomNorm))

  logger.finalize()

  #--- plot ---#
  ax = plt.gca()
  plt.rcParams.update({'font.size': 18})
  ax.plot(fomObj.xGrid, fomFinalState, '-g', linewidth=2, label='FOM')
  ax.plot(fomObj.xGrid, approximatedState, 'or',
              markerfacecolor='None', markersize=5,
              label='LSPG: with kPCA mapping')
  ax.set_ylabel("Solution", fontsize=18)
  ax.set_xlabel("x-coordinate", fontsize=18)
  leg = plt.legend(fontsize=12, fancybox=True, framealpha=0, loc='lower right')
  ax.grid(True, linewidth=0.35, color='gray')

  #used to change color to text and axes
  edit_figure_for_web(ax, leg)
  plt.savefig('demo3.png', dpi=250, transparent=True)
  plt.show()
