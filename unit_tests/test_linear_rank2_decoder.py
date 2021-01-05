
import numpy as np
import math
from pressio4py import rom as rom

def test_when_fortran_layout():
  print("test f layout")
  phi = np.zeros((11,3), order='F')
  phi_addr = phi.__array_interface__['data'][0]
  print("py:phi: ", hex(phi_addr))
  decoder = rom.Decoder(phi)
  addressInsideDecoder = decoder.jacobianAddress()
  print("py:phiFromDec: ", hex(addressInsideDecoder))
  assert( hex(addressInsideDecoder) == hex(phi_addr) )

def test_when_c_layout():
  print("test c layout")
  phi = np.zeros((11,3), order='C')
  phi_addr = phi.__array_interface__['data'][0]
  print("py:phi: ", hex(phi_addr))
  try:
    decoder = rom.Decoder(phi)
  except Exception as e:
    msg = str(e)
    gold = "The linear decoder needs a column-major jacobian but you are passing a row-major one."
    print(msg)
    assert(msg == gold)

def test_mapping_rank1_state():
  print("test mapping rank1 state")
  fomSize, romSize = 6, 3
  phi = np.array([[1, 1, 1],
                  [2, 2, 2],
                  [3, 3, 3],
                  [4, 4, 4],
                  [5, 5, 5],
                  [6, 6, 6]], order='F')

  print(phi.flags['C_CONTIGUOUS'])
  print(phi.flags['F_CONTIGUOUS'])
  decoder = rom.Decoder(phi)
  yRom = np.array([1.,2.,3.])
  yFom = np.zeros(fomSize)
  decoder.applyMapping(yRom, yFom)
  print(yFom)
  assert(math.isclose(yFom[0], 6.))
  assert(math.isclose(yFom[1], 12.))
  assert(math.isclose(yFom[2], 18.))
  assert(math.isclose(yFom[3], 24.))
  assert(math.isclose(yFom[4], 30.))
  assert(math.isclose(yFom[5], 36.))

def test_mapping_rank2_state():
  print("test mapping rank2 state")
  fomSize, romSize, replicas = 6, 3, 2
  phi = np.array([[1, 1, 1],
                  [2, 2, 2],
                  [3, 3, 3],
                  [4, 4, 4],
                  [5, 5, 5],
                  [6, 6, 6]], order='F')

  yRom = np.zeros((romSize, replicas), order='F')
  yFom = np.zeros((fomSize, replicas), order='F')

  for r in range(replicas):
    yRom[:,r] = np.array([1.,2.,3.])

  decoder = rom.rank2state.Decoder(phi)
  decoder.applyMapping(yRom, yFom)
  print(yFom)
  for r in range(replicas):
    assert(math.isclose(yFom[0,r], 6.))
    assert(math.isclose(yFom[1,r], 12.))
    assert(math.isclose(yFom[2,r], 18.))
    assert(math.isclose(yFom[3,r], 24.))
    assert(math.isclose(yFom[4,r], 30.))
    assert(math.isclose(yFom[5,r], 36.))
