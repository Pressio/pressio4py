
import numpy as np
import math
from pressio4py import rom as rom

def test_fom_reconstructor_rank1_state_rank2_mapper():
  print("test_fom_reconstructor_rank1_state_rank2_mapper")
  fomSize, romSize = 6, 3
  phi = np.array([[1, 1, 1],
                  [2, 2, 2],
                  [3, 3, 3],
                  [4, 4, 4],
                  [5, 5, 5],
                  [6, 6, 6]], order='F')

  yFom = np.ones(fomSize)
  yRom = np.array([1.,2.,3.])

  decoder = rom.Decoder(phi)
  fomRec = rom.FomReconstructor(yFom, decoder)
  yFom = fomRec.evaluate(yRom)
  print(yFom)
  assert(yFom[0]==7)
  assert(yFom[1]==13)
  assert(yFom[2]==19)
  assert(yFom[3]==25)
  assert(yFom[4]==31)
  assert(yFom[5]==37)

def test_fom_reconstructor_rank2_state_rank2_mapper():
  print("test_fom_reconstructor_rank2_state_rank2_mapper")
  fomSize, romSize, replicas = 6, 3, 2
  phi = np.array([[1, 1, 1],
                  [2, 2, 2],
                  [3, 3, 3],
                  [4, 4, 4],
                  [5, 5, 5],
                  [6, 6, 6]], order='F')

  yFom = np.ones((fomSize, replicas), order='F')
  yRom = np.zeros((romSize, replicas), order='F')
  yRom[:,0] = np.array([1.,2.,3.])
  yRom[:,1] = np.array([1.,2.,3.])

  decoder = rom.rank2state.Decoder(phi)
  fomRec = rom.rank2state.FomReconstructor(yFom, decoder)
  yFom = fomRec.evaluate(yRom)
  print(yFom)
  for r in range(replicas):
    assert(math.isclose(yFom[0,r], 7.))
    assert(math.isclose(yFom[1,r], 13.))
    assert(math.isclose(yFom[2,r], 19.))
    assert(math.isclose(yFom[3,r], 25.))
    assert(math.isclose(yFom[4,r], 31.))
    assert(math.isclose(yFom[5,r], 37.))

def test_fom_reconstructor_rank2_state_rank3_mapper():
  print("test_fom_reconstructor_rank2_state_rank3_mapper")
  fomSize, romSize, numFields = 6, 3, 2

  phi = np.zeros((fomSize, romSize, numFields), order='F')

  yFom = np.ones((fomSize, numFields), order='F')
  yRom = np.zeros((romSize, numFields), order='F')
  yRom[:,0] = np.array([1.,2.,3.])
  yRom[:,1] = np.array([1.,2.,3.])

  for k in range(numFields):
    for p in range(romSize):
      phi[:,p,k] = np.copy(np.array([1,2,3,4,5,6]))

  decoder = rom.rank2state.MultiFieldDecoder(phi)
  fomRec  = rom.rank2state.MultiFieldFomReconstructor(yFom, decoder)
  yFom    = fomRec.evaluate(yRom)
  print(yFom)
  for r in range(numFields):
    assert(math.isclose(yFom[0,r], 7.))
    assert(math.isclose(yFom[1,r], 13.))
    assert(math.isclose(yFom[2,r], 19.))
    assert(math.isclose(yFom[3,r], 25.))
    assert(math.isclose(yFom[4,r], 31.))
    assert(math.isclose(yFom[5,r], 37.))

def test_reconstructor_rank2_state_rank3_mapper():
  '''
  This test mimics what a fom reconstructor would do:
  Here we test:
    Y = Ax + Yref

  Y, Yref are 6x1x2
  A is 6x3x2
  B is 3x1x2
  '''
  print("test_reconstructor rank3 mapper")
  fomSize, romSize, numFields, replicas = 6, 3, 2, 1
  phi = np.zeros((fomSize, romSize, numFields), order='F')
  yRef = np.ones((fomSize, replicas, numFields), order='F')
  yRom = 2.*np.ones((romSize, replicas, numFields), order='F')

  for k in range(numFields):
    for p in range(romSize):
      phi[:,p,k] = np.copy(np.array([1,2,3,4,5,6]))

  decoder = rom.rank3state.MultiFieldDecoder(phi)
  fomRec  = rom.rank3state.MultiFieldFomReconstructor(yRef, decoder)
  yFom    = fomRec.evaluate(yRom)
  print(yFom)

  gold = np.array([7.,13.,19.,25.,31.,37])
  for k in range(numFields):
    assert(np.allclose(yFom[:,0,k], gold))
