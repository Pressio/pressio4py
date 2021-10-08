
import pytest
import numpy as np
from pressio4py import rom as rom

class MyCustomMap:
  def __init__(self):
    self.phi_ = np.zeros((20,3), order='F')
    phi_addr = self.phi_.__array_interface__['data'][0]
    print("py:cnstr:phi: ", hex(phi_addr))

  def jacobian(self):
    return self.phi_

  def phiAddress(self):
    return hex(self.phi_.__array_interface__['data'][0])

  def applyMapping(self, romState, fomState):
    pass

  def updateJacobian(self, romState):
    self.phi_[:,0] = 1.

def test1():
  mapper = MyCustomMap()
  decoder = rom.Decoder(mapper, "MyMapper")

  # since we use a column-major layout for phi inside mapper,
  # we need to make sure that the decoder views the phi, i.e.
  # the jacobian matrix inside the decoder must
  # have same address as self.phi above
  assert( mapper.phiAddress() == hex(decoder.jacobianAddress()) )
