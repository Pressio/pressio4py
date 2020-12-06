
import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt
import sklearn.decomposition as skd

# need to add to python path location of the apps
import pathlib, sys
file_path = pathlib.Path(__file__).parent.absolute()
sys.path.append(str(file_path) + "/../../apps")
sys.path.append(str(file_path) + "/../..")

from adv_diff1d import *
from pressio4py import rom as rom
from pressio4py import solvers as solvers

#---------------
### FOM ###
#---------------
def test_fom(fom, dt, nsteps, ax):
  u = fom.u0.copy()
  U = [u]
  T = [0.0]
  f = fom.createVelocity()
  for i in range(1,nsteps+1):
    fom.velocity(u, i*dt, f)
    u = u + dt*f
    if i % 200 == 0:
      U.append(u)
      T.append(i*dt)
  Usolns = np.array(U)
  T = np.array(T)
  print("done loop")

  # plot
  x = fom.xGrid
  u0 = Usolns[0]
  uT = Usolns[-1]
  #ax.plot(x,u0,'g',label='initial')
  ax.plot(x,uT,'-k',linewidth=2., label='True solution')# at T=%.3f'%tfinal)

  print("SVD on matrix: ", Usolns.T.shape)
  U,S,VT = np.linalg.svd(Usolns.T)
  np.savetxt("snapshots.txt",Usolns)
  #romSize = 2
  ## POD
  #Phi = U[:,:romSize]
  #uTpod = np.dot(Phi, np.dot(Phi.T, uT))
  #ax.plot(x,uTpod,'-b',label='POD')
  ## KPCA
  #mymap = MyMapperKPCA(romSize)
  #uhat = mymap.applyInverseMapping(uT)
  ##uhat = mymap.transformer_.transform(uT.reshape(1,-1))
  #print(uhat.shape)
  #uTkpca = uT.copy()
  #mymap.applyMapping(uhat,uTkpca)
  #ax.plot(x,uTkpca,'-r',label='KPCA')
  #plt.show()
  nbasis = -1
  np.savetxt("basis.txt", U[:,:nbasis])
  return uT

#----------------------------
class MyMapperKPCA:
  def __init__(self, romSize):
    fname = str("snapshots.txt")
    snapshots = np.loadtxt(fname)
    self.transformer_ = skd.KernelPCA(n_components=romSize,\
                                     kernel='poly',
                                     degree=3,
                                     fit_inverse_transform=True)
    self.transformer_.fit(snapshots)
    self.romSize_ = romSize
    fomSize = snapshots.shape[1]
    self.phi_ = np.zeros((fomSize,romSize), order='F')
    self.fomState0 = np.zeros(fomSize)
    self.fomState1 = np.zeros(fomSize)

  def jacobian(self):
    return self.phi_

  def applyMapping(self, romState, fomState):
    fomState[:] = np.squeeze(self.transformer_.inverse_transform(romState.reshape(1,-1)))

  def applyInverseMapping(self, fomState):
    return np.squeeze(self.transformer_.transform(fomState.reshape(1,-1)))

  def updateJacobian(self, romState):
    romStateLocal = romState.copy()
    # finite difference to approximate jacobian of KPCA
    # compute unperturbed fomState
    self.applyMapping(romStateLocal,self.fomState0)
    eps = 0.001
    for i in range(self.romSize_):
        romStateLocal[i] += eps
        self.applyMapping(romStateLocal, self.fomState1)
        self.phi_[:,i] = (self.fomState1 - self.fomState0) / eps
        romStateLocal[i] -= eps


#----------------------------------------
class OdeObserver:
  def __init__(self, fomRec=None):
    self.fomRec = fomRec

  def __call__(self, timeStep, time, state):
    pass
    # print(state)
    # fs = self.fomRec.evaluate(state)
    # print(fs.shape)
    # assert(fs.shape[0]==50)

#---------------
### LSPG ###
#---------------
class MyLinSolver:
  def __init__(self):
    pass

  def solve(self, A,b,x):
    lumat, piv, info = linalg.lapack.dgetrf(A, overwrite_a=True)
    x[:], info = linalg.lapack.dgetrs(lumat, piv, b, 0, 0)

def test_lspg(appObj, dt, nsteps, romSize, pValue):
  yRef = np.zeros(appObj.nGrid)
  mymap   = MyMapperKPCA(romSize)
  decoder = rom.Decoder(mymap, "MyMapperKPCA")

  # project fom initial state to compute rom initial state
  yFom0 = appObj.u0.copy()
  yRom = mymap.applyInverseMapping(yFom0)
  mymap.updateJacobian(yRom)

  problem = rom.lspg.unsteady.default.ProblemEuler(appObj, decoder, yRom, yRef)
  fomRecon = problem.fomStateReconstructor()

  # linear and non linear solver
  if (pValue==-1):
    nlsO = solvers.GaussNewton(problem, yRom, MyLinSolver())
  else:
    nlsO = solvers.IrwGaussNewton(problem, yRom, MyLinSolver(), pValue)
  nlsTol, nlsMaxIt = 1e-7, 6
  nlsO.setMaxIterations(nlsMaxIt)
  nlsO.setStoppingCriterion(solvers.stop.whenCorrectionAbsoluteNormBelowTolerance)
  #nlsO.setStoppingCriterion(solvers.stop.afterMaxIters)
  nlsO.setCorrectionAbsoluteTolerance(nlsTol)

  # solve
  myObs = OdeObserver()
  rom.lspg.solveNSequentialMinimizations(problem, yRom, 0.,dt, nsteps, myObs, nlsO)
  yFomFinal = fomRecon.evaluate(yRom)
  return yFomFinal

#########################
######## MAIN ###########
#########################
if __name__ == "__main__":
  # initial condition
  ic = lambda x: 2.*np.sin(9.*np.pi*x) - np.sin(4.*np.pi*x)
  # create fom object
  fom = AdvDiff1d(nGrid=120, IC=ic, adv_coef=2.0)

  tfinal = .05
  ax = plt.gca()

  # FOM #
  dtFom = 1e-5
  gold = test_fom(fom, dtFom, int(tfinal/dtFom), ax)

  # ROM #
  dtRom = 3e-4
  nsteps = int(tfinal/dtRom)
  romSizes = [2,3,4]
  colors  = ['b', 'r', 'y']
  for p,c in zip(romSizes,colors):
   yFomLSPG = test_lspg(fom, dtRom, nsteps, p, -1)
   goldNorm = linalg.norm(gold)
   en = linalg.norm(gold-yFomLSPG)
   print("LSPG p= {} err: {}, {}".format(p, en, en/goldNorm))
   ax.plot(fom.xGrid, yFomLSPG, '-o',
           markerfacecolor='None', markeredgecolor=c,
           color=c,
           label="p="+str(p))

  plt.rcParams.update({'font.size': 18})
  plt.ylabel("Solution", fontsize=18)
  plt.xlabel("x-coordinate", fontsize=18)
  plt.legend(fontsize=12)
  plt.show()
