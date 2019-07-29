
import numpy as np
import pressio4py
import burgers
import pressio4pyOps

np.set_printoptions(precision=15)
Ncell = 20
mu = np.array([5., 0.02, 0.02])
appObj = burgers.Burgers1d(mu,Ncell)
appObj.setup()

# reference state
yRef = np.ones(Ncell)

# object in charge of ops
ops = pressio4pyOps.Ops()

# load basis into numpy array
phi = np.loadtxt("basis.txt")

# create a decoder
decoder = pressio4py.LinearDecoder(phi, ops)

# the LSPG (reduced) state
romSize = phi.shape[1]
yRom = np.zeros(romSize)

# the problem
t0 = 0.
lspgObj = pressio4py.LspgProblem(appObj, yRef, decoder, yRom, t0, ops)

# get stepper
stepper = lspgObj.getStepper()

# linear solver
lsO = pressio4pyOps.LinSolver()

# non linear solver
nlsO = pressio4py.GaussNewton(stepper, yRom, lsO, ops)
print( nlsO.getMaxIterations() )
print( nlsO.getTolerance() )
nlsO.setMaxIterations(20)
nlsO.setTolerance(1e-13)

dt, nsteps = 0.01, 10
pressio4py.integrateNSteps(stepper, yRom, 0.0, dt, nsteps, nlsO)

goldGenCoords = np.array([-0.24149601775247,
                          -0.00058769865883,
                          0.00000072723063,
                          0.00000000100402,
                          0.00000000000580,
                          -0.00000000000822,
                          0.00000000000105,
                          -0.00000000000118,
                          -0.00000000000021,
                          -0.00000000000643,
                          -0.00000000000362])
errVec = goldGenCoords-yRom
mynorm = np.linalg.norm(errVec)
assert(mynorm < 1e-13)
assert( np.max(errVec) < 1e-13)
print(yRom-goldGenCoords)
