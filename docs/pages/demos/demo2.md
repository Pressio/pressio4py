
# 1D adv-diff: LSPG with POD modes


@m_class{m-block m-info}

@par What does this page describe?
This page describes a demo for a reproductive LSPG ROM applied to a
1D advection-diffusion problem using POD modes as basis.
By the end, it should be clear how to setup the problem and
the various steps involved.<br>
To jump directly at the full demo script, click [here.](https://github.com/Pressio/pressio4py/blob/master/demos/unsteady_default_lspg_advdiff1d_pod/main.py)


@m_class{m-block m-warning}

@par Disclaimer
This demo purposefully focuses on a simple test since the main goal is
to demonstrate the steps and the code. More complex cases will be added later.


## Overview
Here, we cover the three typical steps needed for a ROM:
1. generating of snapshots using the full-order model (FOM)
2. computing the basis: here we demonstrate the use of POD modes
3. executing the ROM: here we leverage the LSPG ROM to demonstrate
a *reproductive* test, i.e., we run the ROM using the same physical coefficients, b.c., etc.
A predictive run is demonstrated in a different tutorial.

## FOM Equations
The governing equations for this problem are:

@f[
\frac{\partial u}{\partial t}
= \frac{\partial}{\partial x} (k(u,x) \frac{\partial u}{\partial x} )
- a*\frac{\partial u}{\partial x}
@f]
where @f$k(u,x)=x^4@f$, the field is @f$u(x;t)@f$, the advection velocity
is fixed at @f$a=2@f$, the spatial coordinate is @f$x@f$ and the domain is @f$(0,1)@f$.
We use homogeneous BC. Note that a class approximating the FOM operators via finite-differences
is implemented [here](https://github.com/Pressio/pressio4py/blob/master/apps/adv_diff1d.py).


## Imports
Before looking at the code snippets below, the imports needed are:
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

  #--- 2. POD ---#
  modes = computePodModes(snapshots)

  #--- 3. LSPG ROM ---#
  romSize = 4
  romTimeStepSize  = 3e-4
  romNumberOfSteps = int(finalTime/romTimeStepSize)
  # we pass only romSize modes
  approximatedState = runLspg(fomObj, romTimeStepSize,
                              romNumberOfSteps, modes[:,:romSize])

  # compute l2-error between fom and approximate state
  fomNorm = linalg.norm(fomFinalState)
  err = linalg.norm(fomFinalState-approximatedState)
  print("Final state relative l2 error: {}".format(err/fomNorm))

  #--- plot ---#
  ax = plt.gca()
  ax.plot(fomObj.xGrid, fomFinalState, '-', linewidth=2, label='FOM')
  ax.plot(fomObj.xGrid, approximatedState, 'or', label='LSPG: '+str(romSize)+' POD modes')
  plt.rcParams.update({'font.size': 18})
  plt.ylabel("Solution", fontsize=18)
  plt.xlabel("x-coordinate", fontsize=18)
  plt.legend(fontsize=12)
  plt.show()
```

## Code for the various stages in main
Here we list the functions performing the various stages of the run.

### 1. Run FOM and collect snapshots
```py
def doFom(fom, dt, nsteps, saveFreq):
  u = fom.u0.copy()
  U = [u]
  T = [0.0]
  f = fom.createVelocity()
  for i in range(1,nsteps+1):
    # query rhs of discretized system
    fom.velocity(u, i*dt, f)
    # simple Euler forward
    u = u + dt*f
    if i % saveFreq == 0:
      U.append(u)
      T.append(i*dt)
  Usolns = np.array(U)
  return [u, Usolns.T]
```

### 2. Compute POD modes
```py
def computePodModes(snapshots):
  print("SVD on matrix: ", snapshots.shape)
  U,S,VT = np.linalg.svd(snapshots)
  return U
```

### 3. Construct and run LSPG
```py
def runLspg(fomObj, dt, nsteps, modes):
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

  # create LSPG problem
  problem = rom.lspg.unsteady.default.ProblemEuler(fomObj, linearDecoder, romState, fomReferenceState)

  # create the Gauss-Newton solver
  nonLinSolver = solvers.GaussNewton(problem, romState, MyLinSolver())
  # set tolerance and convergence criteria
  nlsTol, nlsMaxIt = 1e-6, 5
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
