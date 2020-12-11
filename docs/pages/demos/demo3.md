
# 1D adv-diff: LSPG with nonlinear manifold projection via kPCA


@m_class{m-block m-info}

@par What does this page describe?
This page describes a demo for a reproductive LSPG ROM applied to a
1D advection-diffusion problem using a nonlinear manifold via kernel PCA.
This demo purposefully focuses on a simple test since the main goal is
to demonstrate the steps and the code. More complex cases will be added later.
To jump directly at the full demo script, click [here.](https://github.com/Pressio/pressio4py/blob/master/demos/unsteady_default_lspg_advdiff1d_kpca/main.py)


## Overview

This demo solves the same problem as the one [here](https://pressio.github.io/pressio4py/html/md_pages_demos_demo2.html),
but instead of using POD modes, we show here how to use a nonlinear manifold computed via kernel PCA.

## Imports
Before looking at the code snippets below, the `pressio4py`-specific imports needed are:
```py
from adv_diff1d import *					# the fom class
from adv_diff_1d_fom import doFom			# the function to collect fom data
from pressio4py import rom as rom
from pressio4py import solvers as solvers

```

## Main function
The main function of the demo is the following:
```py
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

  #--- plot ---#
  ax = plt.gca()
  ax.plot(fomObj.xGrid, fomFinalState, '-', linewidth=2, label='FOM')
  ax.plot(fomObj.xGrid, approximatedState, 'or', label='LSPG: with kPCA mapping')
  plt.rcParams.update({'font.size': 18})
  plt.ylabel("Solution", fontsize=18)
  plt.xlabel("x-coordinate", fontsize=18)
  plt.legend(fontsize=12)
  plt.show()
```

## Code for the various stages in main
Here we list the functions performing the various stages of the run.

### 1. Run FOM and collect snapshots
This step is the same as described [here](https://pressio.github.io/pressio4py/html/md_pages_demos_demo2.html),


### 2. Setup and train the nonlinear kPCA mapper
It is important to note that while the mapper class below has
the API required by pressio4py, it can encapsulate any arbitrary mapping function.
In this case we show how to create a kPCA-based representation, but one
can use, e.g., autoencoder, and any other types of mapping.
This is how we enable support for testing various methods.
```py
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

  def jacobian(self): return self.jacobian_

  def applyMapping(self, romState, fomState):
    fomState[:] = np.squeeze(self.transformer_.inverse_transform(romState.reshape(1,-1)))

  def applyInverseMapping(self, fomState):
    return np.squeeze(self.transformer_.transform(fomState.reshape(1,-1)))

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
```
@m_class{m-block m-warning}

@par Important:
when creating an arbitrary mapping (as in the class above),
the jacobian matrix **must** be column-major oder so that pressio
can reference it without deep copying it. This not only reduced the
memory footprint since it allows to keep only one jacobian object
around but also it is fundamental for the update method below correctly.



### 3. Construct and run LSPG
```py
def runLspg(fomObj, dt, nsteps, customMapper):
  # this is an auxiliary class that can be passed to solve
  # LSPG to monitor the rom state.
  class RomStateObserver:
    def __init__(self): pass
    def __call__(self, timeStep, time, state): pass
  #----------------------------------------
  # this linear solver is used at each gauss-newton iteration
  class MyLinSolver:
    def __init__(self): pass
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
  romState = customMapper.applyInverseMapping(fomInitialState)

  # create LSPG problem
  problem = rom.lspg.unsteady.default.ProblemEuler(fomObj, customDecoder, romState, fomReferenceState)

  # create the Gauss-Newton solver
  nonLinSolver = solvers.GaussNewton(problem, romState, MyLinSolver())
  # set tolerance and convergence criteria
  nlsTol, nlsMaxIt = 1e-7, 10
  nonLinSolver.setMaxIterations(nlsMaxIt)
  nonLinSolver.setStoppingCriterion(solvers.stop.whenCorrectionAbsoluteNormBelowTolerance)

  # create object to monitor the romState at every iteration
  myObs = RomStateObserver()
  # solver LSPG problems
  rom.lspg.solveNSequentialMinimizations(problem, romState, 0., dt, nsteps, myObs, nonLinSolver)

  # after we are done, use the reconstructor object to reconstruct the fom state
  # get the reconstructor object: this allows to map romState to fomState
  fomRecon = problem.fomStateReconstructor()
  return fomRecon.evaluate(romState)
```