
import numpy as np
from scipy import linalg

from burgers1d_sparse_jacobian import Burgers1dSparseJacobian
from pressio4py import rom as rom

def test_euler():
  meshSize = 50
  romSize  = 20
  Nsteps   = 3500
  dt       = 0.01

  appObj = Burgers1dSparseJacobian(meshSize)
  # reference state
  yRef = np.ones(meshSize)
  # basis
  phi = np.loadtxt("./burgers1d_galerkin/basis.txt")
  decoder = rom.LinearDecoder(phi)
  yRom = np.zeros(romSize)

  galerkinObj = rom.galerkin.ProblemEuler(appObj, yRef, decoder, yRom, 0.)
  stepper = galerkinObj.getStepper()
  rom.galerkin.advanceNStepsEuler(stepper, yRom, 0., dt, Nsteps)
  # np.savetxt("final_generalized_coords.txt", yRom, fmt='%.16f')

  fomRecon = galerkinObj.getFomStateReconstructor()
  yFomFinal = fomRecon.evaluate(yRom)
  print(yFomFinal)

  gold = np.array([5.0081542681376, 5.016629490569, 5.025433912557,
                   5.0345792953115,5.0440827179355,5.0539568295087,5.0642107801363,
                   5.074857742734,5.0859146000515,5.0974001265619,5.1093302710968,
                   5.1217197481536,5.1345846667293,5.1479436063682,5.1618137609004,
                   5.1762071980595,5.1911395190849,5.2066322357211, 5.222706587389,
                   5.2393822195142,5.2566784890019, 5.274617970535,5.2932246323729,
                   5.3125186218141,5.3325236627322,5.3532729201416,5.3747971779128,
                   5.3971189932731,5.4202577535351,5.4442348269811, 5.469078757402,
                   5.4948202159561,5.5214859714822,5.5491009348394,5.5776911098501,
                   5.6072849195866,5.6379131952825,5.6696069037791,5.7023980878343,
                   5.7363239274031,5.7714263431002, 5.807744410524,5.8453128737429,
                   5.884168702448,5.9243510856362,5.9658923478856,6.0088164545724,
                   6.0531503069487,6.0989210765093,6.1461565470309])

  for y1,y2 in zip(gold, yFomFinal):
    assert( np.abs(y1-y2) < 1e-12)
