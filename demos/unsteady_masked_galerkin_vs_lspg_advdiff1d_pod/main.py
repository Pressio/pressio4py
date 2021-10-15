
import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt
import random

# if run from within a build of pressio4py, need to append to python path
import pathlib, sys
file_path = pathlib.Path(__file__).parent.absolute()
sys.path.append(str(file_path) + "/..")         # to access doFom

from pressio4py import logger, solvers, ode, rom
from pressio4py.apps.advection_diffusion1d import AdvDiff1d
from adv_diff_1d_fom import doFom
from settings_for_website import edit_figure_for_web

#----------------------------
class MyLinSolver:
  def solve(self, A,b,x):
    lumat, piv, info = linalg.lapack.dgetrf(A, overwrite_a=True)
    # here we must use x[:] otherwise it won't overwrite x passed in
    x[:], info = linalg.lapack.dgetrs(lumat, piv, b, 0, 0)

#----------------------------
class MyMasker:
  def __init__(self, indices):
    self.rows_ = indices
    self.sampleMeshSize_ = len(indices)

  def createApplyMaskResult(self, operand):
    if (operand.ndim == 1):
      return np.zeros(self.sampleMeshSize_)
    else:
      return np.zeros((self.sampleMeshSize_, operand.shape[1]))

  def __call__(self, operand, time, result):
    if (operand.ndim == 1):
      result[:] = np.take(operand, self.rows_)
    else:
      result[:] = np.take(operand, self.rows_, axis=0)

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
  scheme = ode.stepscheme.BDF1
  problem = rom.galerkin.MaskedImplicitProblem(scheme, fomObj, linearDecoder, \
                                               romState, fomReferenceState, \
                                               projector, masker)

  # linear and non linear solver
  lsO = MyLinSolver()
  nlsO = solvers.create_newton_raphson(problem, romState, lsO)
  nlsO.setMaxIterations(15)

  # solve the problem
  ode.advance_n_steps(problem, romState, 0., dt, nsteps, nlsO)

  # after we are done, use the reconstructor object to reconstruct the fom state
  # NOTE: even though the Galerkin problem was run on the "masked mesh points",
  # this reconstruction uses the POD modes on the full mesh stored in the decoder
  # so we can effectively obtain an approximation of the full solution
  fomRecon = problem.fomStateReconstructor()
  return [fomRecon(romState), romState]

#----------------------------------------
def runMaskedLspg(fomObj, dt, nsteps, modes, sampleMeshIndices):
  # find out number of modes wanted
  romSize = modes.shape[1]

  # fom reference state: here it is zero
  fomReferenceState = np.zeros(fomObj.nGrid)

  # create ROM state by projecting the fom initial condition
  fomInitialState = fomObj.u0.copy()
  romState = np.dot(modes.T, fomInitialState)

  '''
  creating a masked LSPG problem involves these steps:
  (1) creating the decoder on the FULL mesh
  (2) create a masker object responsible to mask the FOM operators
  (3) create the masked LSPG problem
  '''

  # 1. create a linear decoder
  linearDecoder = rom.Decoder(modes)

  # 2. create the masker object
  masker = MyMasker(sampleMeshIndices)

  # 3. create the masked galerkin problem with Euler forward
  scheme = ode.stepscheme.BDF1
  problem = rom.lspg.unsteady.MaskedProblem(scheme, fomObj, linearDecoder,\
                                            romState, fomReferenceState,\
                                            masker)

  # linear and non linear solver
  lsO = MyLinSolver()
  nlsO = solvers.create_gauss_newton(problem, romState, lsO)
  nlsO.setMaxIterations(10)

  # solve the problem
  ode.advance_n_steps(problem, romState, 0., dt, nsteps, nlsO)

  # after we are done, use the reconstructor object to reconstruct the fom state
  # NOTE: even though the Galerkin problem was run on the "masked mesh points",
  # this reconstruction uses the POD modes on the full mesh stored in the decoder
  # so we can effectively obtain an approximation of the full solution
  fomRecon = problem.fomStateReconstructor()
  return [fomRecon(romState), romState]

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

  #--- 3. MASKED GALERKIN and LSPG ROM ---#
  # a masked problem is supposed to make it easier to emulate the
  # effect of hyper-reduction. To create a mask ROM problem,
  # we need to select and provide to pressio a set of indices
  # identifying a subset of the grid points in the full mesh.
  # This is a simple way to mimic hyper-reduction
  # without changing the FOM problem. In fact, the fom still
  # computes the full operators but we have an additional step
  # to "mask" the operators to compute the sample mesh version.
  # In this test, the meshSize = 200. Our sample mesh includes
  # the two end points since those contain the boundary conditions,
  # and 20 randomly selected grid points inside the domain.
  # So effectively we use 1/10 of the full mesh.
  random.seed(22123)
  sampleMeshSize = 20
  sampleMeshIndices = random.sample(range(1, 199), sampleMeshSize)
  sampleMeshIndices = np.append(sampleMeshIndices, [0, 199])
  # sort for convenience, not necessarily needed
  sampleMeshIndices = np.sort(sampleMeshIndices)

  romSize = 5  # number of modes to use
  romTimeStepSize  = 1e-4
  romNumberOfSteps = int(finalTime/romTimeStepSize)

  # run the masked galerkin problem
  [approximatedStateGal, romGal] = runMaskedGalerkin(fomObj, romTimeStepSize,
                                                     romNumberOfSteps,
                                                     modes[:,:romSize],
                                                     sampleMeshIndices)
  # run the masked galerkin problem
  [approximatedStateLspg, romLspg] = runMaskedLspg(fomObj, romTimeStepSize,
                                                   romNumberOfSteps,
                                                   modes[:,:romSize],
                                                   sampleMeshIndices)

  # compute l2-error between fom and approximate state
  fomNorm = linalg.norm(fomFinalState)
  err1 = linalg.norm(fomFinalState-approximatedStateGal)
  print("Galerkin: final state relative l2 error: {}".format(err1/fomNorm))
  err2 = linalg.norm(fomFinalState-approximatedStateLspg)
  print("LSPG: final state relative l2 error: {}".format(err2/fomNorm))

  logger.finalize()

  #----------------------------------------------#
  #--- plot solutions on sample and full mesh ---#

  gridSM = fomObj.xGrid[np.sort(sampleMeshIndices)]
  phiSM = np.take(modes[:,:romSize], sampleMeshIndices, axis=0)
  # approxStateSM = phi_sm * romState (no need for reference state since = 0)
  approxStateGalSM  = np.dot(phiSM, romGal)
  approxStateLspgSM = np.dot(phiSM, romLspg)

  fig = plt.figure(0)
  ax = fig.gca()
  #plt.rcParams.update({'font.size': 18})
  ax.plot(fomObj.xGrid, fomFinalState, '-g', linewidth=1.5, label='FOM')

  ax.plot(gridSM, approxStateLspgSM, 'sy',
          markerfacecolor='None', markersize=7,
          label='LSPG solution on sample mesh')
  ax.plot(gridSM, approxStateGalSM, 'sm',
          markerfacecolor='None', markersize=7,
          label='Galerkin solution on sample mesh')
  ax.set_ylabel("Solution")
  ax.set_xlabel("x-coordinate")
  leg = plt.legend(fontsize=10, fancybox=True, framealpha=0, loc='lower right')
  ax.grid(True, linewidth=0.35, color='gray')
  #used to change color to text and axes
  edit_figure_for_web(ax, leg)
  plt.savefig('demo5_f1.png', dpi=250, transparent=True)
  plt.show()

  fig = plt.figure(1)
  ax1 = fig.gca()
  #plt.rcParams.update({'font.size': 18})
  ax1.plot(fomObj.xGrid, fomFinalState, '-g', linewidth=1.5, label='FOM')
  ax1.plot(fomObj.xGrid, approximatedStateLspg, 'oy',
          markerfacecolor='None', markersize=4,
          label='LSPG on full mesh')
  ax1.plot(fomObj.xGrid, approximatedStateGal, 'om',
          markerfacecolor='None', markersize=4,
          label='Galerkin solution on full mesh')

  ax1.set_ylabel("Solution")
  ax1.set_xlabel("x-coordinate")
  leg = plt.legend(fontsize=10, fancybox=True, framealpha=0, loc='lower right')
  ax1.grid(True, linewidth=0.35, color='gray')

  #used to change color to text and axes
  edit_figure_for_web(ax1, leg)
  plt.savefig('demo5_f2.png', dpi=250, transparent=True)
  plt.show()
