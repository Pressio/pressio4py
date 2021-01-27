
import numpy as np
from pressio4py import rom as rom

if __name__ == "__main__":
  class CustomMapper:
    def __init__(self, fomSize, romSize):
      # attention: the jacobian of the mapping must be column-major oder
      # so that pressio can view it without deep copying it, this enables
      # to keep only one jacobian object around and to call the update
      # method below correctly
      self.jacobian_ = np.zeros((fomSize,romSize), order='F')

    def jacobian(self): return self.jacobian_

    def applyMapping(self, romState, fomState):
      #fomState[:] = whatever is needed
      pass

    def updateJacobian(self, romState):
      # update the self.jacobian_[:,:]
      pass

  # create the mapper
  myMapper = CustomMapper(10,3)
  # to create a custom decoder, one can do
  customDecoder = rom.Decoder(myMapper, "MyMapper")
