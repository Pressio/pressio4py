
import numpy as np
from pressio4py import rom as rom

def test_when_fortran_layout():
  print("test f layout")
  phi = np.zeros((11,3,2), order='F')
  phi_addr = phi.__array_interface__['data'][0]
  print("\n")
  print("py:phi: ", hex(phi_addr))
  decoder = rom.rank3state.MultiFieldDecoder(phi)
  addressInsideDecoder = decoder.jacobianAddress()
  print("py:phiFromDec: ", hex(addressInsideDecoder))
  assert( hex(addressInsideDecoder) == hex(phi_addr) )

def test_when_c_layout():
  print("test c layout")
  phi = np.zeros((11,3,5), order='C')
  phi_addr = phi.__array_interface__['data'][0]
  print("py:phi: ", hex(phi_addr))
  try:
    decoder = rom.rank3state.MultiFieldDecoder(phi)
  except Exception as e:
    msg = str(e)
    gold = "The linear decoder needs a column-major jacobian but you are passing a row-major one."
    print(msg)
    assert(msg == gold)

def test_mapping_rank2_state():
  print("testing mapping for rank3 decoder with rank2 state")

  fomSize, romSize, numFields = 6, 3, 2
  phi = np.zeros((fomSize, romSize, numFields), order='F')
  yRom = np.zeros((romSize, numFields), order='F')
  yFom = np.zeros((fomSize, numFields), order='F')

  # set phi = 1 for field=0 and phi=2 for field=1
  phi[:,:,0] = 1.
  phi[:,:,1] = 2.

  yRom[:,0] = 1.1
  yRom[:,1] = 2.1

  decoder = rom.rank2state.MultiFieldDecoder(phi)
  decoder.applyMapping(yRom, yFom)
  print(yFom.shape)
  print(yFom)

  gold = np.zeros((fomSize, numFields))
  gold[:,0] = 3.3
  gold[:,1] = 12.6
  np.testing.assert_allclose(yFom, gold)


def test_mapping_rank3_state():
  print("testing mapping for rank3 decoder with rank3 state")

  fomSize, romSize, numFields = 6, 3, 2
  phi = np.zeros((fomSize, romSize, numFields), order='F')
  yRom = np.zeros((romSize, 1, numFields), order='F')
  yFom = np.zeros((fomSize, 1, numFields), order='F')

  # set phi = 1 for field=0 and phi=2 for field=1
  phi[:,:,0] = 1.
  phi[:,:,1] = 2.

  yRom[:,0,0] = 1.1
  yRom[:,0,1] = 2.1

  decoder = rom.rank3state.MultiFieldDecoder(phi)
  decoder.applyMapping(yRom, yFom)
  print(yFom.shape)
  print(yFom)
  gold = np.zeros((fomSize, 1, numFields))
  gold[:,0,0] = 3.3
  gold[:,0,1] = 12.6
  np.testing.assert_allclose(yFom, gold)
