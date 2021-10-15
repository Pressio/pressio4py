
import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt
import random

# if run from within a build of pressio4py, need to append to python path
import pathlib, sys
file_path = pathlib.Path(__file__).parent.absolute()
sys.path.append(str(file_path) + "/..") # to access doFom

from pressio4py import logger, solvers, ode, rom
from pressio4py.apps.advection_diffusion1d import AdvDiff1d
from adv_diff_1d_fom import doFom
from settings_for_website import edit_figure_for_web

#----------------------------
class MyMasker:
  def __init__(self, indices):
    self.rows_ = indices
    self.sampleMeshSize_ = len(indices)

  def createApplyMaskResult(self, operand):
    return np.zeros(self.sampleMeshSize_)

  def __call__(self, operand, time, result):
    result[:] = np.take(operand, self.rows_)

#----------------------------------------
def computePodModes(snapshots):
  print("SVD on matrix: ", snapshots.shape)
  U,S,VT = np.linalg.svd(snapshots)
  return U

#----------------------------------------
class MyProjector:
  def __init__(self, phi):
    self.phi_ = phi

  def __call__(self, operand, time, result):
    result[:] = np.dot(self.phi_.T, operand)

#----------------------------------------
def runMaskedGalerkin(fomObj, dt, nsteps, modes, sampleMeshIndices):
  # find out number of modes wanted
  romSize = modes.shape[1]

  # fom reference state: here it is zero
  fomReferenceState = np.zeros(fomObj.nGrid)

  # create ROM state by projecting the fom initial condition
  fomInitialState = fomObj.u0.copy()
  romState = np.dot(modes.T, fomInitialState)

  '''
  creating a masked Galerkin problem involves these steps:
  (1) creating the decoder on the FULL mesh
  (2) create a "projector operator" by filtering the rows
      of the POD modes only on the sample mesh (aka mask) indices.
      The projector is responsible to project the FOM velocity.
      Note that one can use other matrices for the projector
      but that will be shown in other demos.
  (3) create a masker object responsible to mask the FOM operators.
  (4) create the masked Galerkin problem
  '''

  # 1. create a linear decoder
  linearDecoder = rom.Decoder(modes)

  # 2. create the projector
  # here, simply use "collocation" with the POD modes filtered on the "sample mesh"
  modesOnSampleMesh = np.take(modes, sampleMeshIndices, axis=0)
  projector = MyProjector(modesOnSampleMesh)

  # 3. create the masker object
  masker = MyMasker(sampleMeshIndices)

  # 4. create the masked galerkin problem with Euler forward
  scheme = ode.stepscheme.ForwardEuler
  problem = rom.galerkin.MaskedExplicitProblem(scheme, fomObj, linearDecoder, \
                                               romState, fomReferenceState, \
                                               projector, masker)

  # solve problem
  ode.advance_n_steps(problem, romState, 0., dt, nsteps)

  # after we are done, use the reconstructor object to reconstruct the fom state
  # NOTE: even though the Galerkin problem was run on the "masked mesh points",
  # this reconstruction uses the POD modes on the full mesh stored in the decoder
  # so we can effectively obtain an approximation of the full solution
  fomRecon = problem.fomStateReconstructor()
  return fomRecon(romState)

######## MAIN ###########
if __name__ == "__main__":
  logger.initialize(logger.logto.terminal)
  logger.setVerbosity([logger.loglevel.info])

  # total number of grid points
  meshSize = 200

  # create fom object
  fomObj = AdvDiff1d(nGrid=meshSize, adv_coef=1.0)

  # the final time to integrate to
  finalTime = .05

  #--- 1. FOM ---#
  fomTimeStepSize  = 1e-5
  fomNumberOfSteps = int(finalTime/fomTimeStepSize)
  sampleEvery      = 100
  [fomFinalState, snapshots] = doFom(fomObj, fomTimeStepSize, fomNumberOfSteps, sampleEvery)

  #--- 2. POD ---#
  modes = computePodModes(snapshots)

  #--- 3. MASKED GALERKIN ROM ---#
  romSize = 10  # number of modes to use
  romTimeStepSize  = 1e-4
  romNumberOfSteps = int(finalTime/romTimeStepSize)

  # a masked galerkin is supposed to make it easier to emulate the
  # effect of hyper-reduction. To create a mask ROM problem,
  # we need to select and provide to pressio a set of indices
  # identifying a subset of the grid points in the full mesh.
  # This is a simple way to mimic hyper-reduction
  # without changing the FOM problem. In fact, the fom still
  # computes the full operators but we have an additional step
  # to "mask" the operators to compute the sample mesh version.
  # In this test, the meshSize = 200. Our sample mesh includes
  # the two end points since those contain the boundary conditions,
  # and 150 randomly selected grid points inside the domain.
  # So effectively we use 25% less of the full mesh.
  random.seed(312367)
  sampleMeshSize = 150
  sampleMeshIndices = random.sample(range(1, 199), sampleMeshSize)
  sampleMeshIndices = np.append(sampleMeshIndices, [0, 199])

  # run the masked galerkin problem
  approximatedState = runMaskedGalerkin(fomObj, romTimeStepSize,
                                        romNumberOfSteps, modes[:,:romSize],
                                        sampleMeshIndices)

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
          label='Galerkin: '+str(romSize)+' POD modes')
  ax.set_ylabel("Solution")
  ax.set_xlabel("x-coordinate")
  leg = plt.legend(fontsize=12, fancybox=True, framealpha=0, loc='lower right')
  ax.grid(True, linewidth=0.35, color='gray')

  #used to change color to text and axes
  edit_figure_for_web(ax, leg)
  plt.savefig('demo4.png', dpi=250, transparent=True)
  plt.show()
