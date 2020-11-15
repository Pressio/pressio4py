
import numpy as np
from scipy import linalg

import pathlib, sys
file_path = pathlib.Path(__file__).parent.absolute()
sys.path.append(str(file_path) + "/../apps")

from burgers1d_sparse_jacobian import Burgers1dSparseJacobian
from pressio4py import rom as rom

gold = np.array(
  [5.0081542681376, 5.016629490569,
   5.025433912557,  5.0345792953115,
   5.0440827179355, 5.0539568295087,
   5.0642107801363, 5.074857742734,
   5.0859146000515, 5.0974001265619,
   5.1093302710968, 5.1217197481536,
   5.1345846667293, 5.1479436063682,
   5.1618137609004, 5.1762071980595,
   5.1911395190849, 5.2066322357211,
   5.222706587389,  5.2393822195142,
   5.2566784890019, 5.274617970535,
   5.2932246323729, 5.3125186218141,
   5.3325236627322, 5.3532729201416,
   5.3747971779128, 5.3971189932731,
   5.4202577535351, 5.4442348269811,
   5.469078757402,  5.4948202159561,
   5.5214859714822, 5.5491009348394,
   5.5776911098501, 5.6072849195866,
   5.6379131952825, 5.6696069037791,
   5.7023980878343, 5.7363239274031,
   5.7714263431002, 5.807744410524,
   5.8453128737429, 5.884168702448,
   5.9243510856362, 5.9658923478856,
   6.0088164545724, 6.0531503069487,
   6.0989210765093, 6.1461565470309])

#----------------------------------------
class OdeObserver:
  def __init__(self, fomRec):
    self.fomRec = fomRec

  def __call__(self, timeStep, time, state):
    print(state)
    fs = self.fomRec.evaluate(state)
    print(fs.shape)
    assert(fs.shape[0]==50)

#----------------------------------------
def test_euler():
  meshSize = 50
  romSize  = 20
  Nsteps   = 3500
  dt       = 0.01

  # create fom object
  appObj = Burgers1dSparseJacobian(meshSize)
  # reference state
  yRef = np.ones(meshSize)
  # basis
  phi = np.loadtxt("./burgers1d_galerkin/basis.txt")

  decoder = rom.Decoder(phi)

  # create rom state
  yRom = np.zeros(romSize)
  # create problem
  galerkinProblem = rom.galerkin.default.ProblemEuler(appObj, decoder, yRom, yRef)

  fomRecon = galerkinProblem.fomStateReconstructor()
  # the observer is called to monitor evolution of rom_state and
  # uses the reconstructor object to reconstruct FOM state
  myObs = OdeObserver(fomRecon)
  rom.galerkin.advanceNSteps(galerkinProblem, yRom, 0., dt, Nsteps, myObs)

  # reconstruct full state at the end
  yFomFinal = fomRecon.evaluate(yRom)
  print(yFomFinal)

  # check solution is right
  for y1,y2 in zip(gold, yFomFinal):
    assert( np.abs(y1-y2) < 1e-12)
