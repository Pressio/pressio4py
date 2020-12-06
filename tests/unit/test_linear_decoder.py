
import numpy as np
from pressio4py import rom as rom

def test_when_fortran_layout():
  phi = np.zeros((11,3), order='F')
  phi_addr = phi.__array_interface__['data'][0]
  print("\n")
  print("py:phi: ", hex(phi_addr))
  decoder = rom.Decoder(phi)
  addressInsideDecoder = decoder.jacobianAddress()
  print("py:phiFromDec: ", hex(addressInsideDecoder))
  assert( hex(addressInsideDecoder) == hex(phi_addr) )

def test_when_c_layout():
  phi = np.zeros((11,3), order='C')
  phi_addr = phi.__array_interface__['data'][0]
  print("\n")
  print("py:phi: ", hex(phi_addr))
  decoder = rom.Decoder(phi)
  addressInsideDecoder = decoder.jacobianAddress()
  print("py:phiFromDec: ", hex(addressInsideDecoder))
  assert( hex(addressInsideDecoder) != hex(phi_addr) )

def test_mapping():
  fomSize, romSize = 6, 3
  phi = np.array([[1, 1, 1],
                  [2, 2, 2],
                  [3, 3, 3],
                  [4, 4, 4],
                  [5, 5, 5],
                  [6, 6, 6]])

  print("test mapping")
  decoder = rom.Decoder(phi)
  yRom = np.array([1.,2.,3.])

  yFom = np.zeros(fomSize)
  decoder.applyMapping(yRom, yFom)
  print(yFom)
  assert(yFom[0]==6)
  assert(yFom[1]==12)
  assert(yFom[2]==18)
  assert(yFom[3]==24)
  assert(yFom[4]==30)
  assert(yFom[5]==36)
