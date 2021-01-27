
import numpy as np
from pressio4py import rom as rom

def rank1StateDecoder():
  # create the matrix
  # attention: we declare phi to be column-major for these reasons:
  #
  # 1. pressio4py uses blas (wherever possible) to operate on numpy arrays,
  #    so a column-major layout implies seamless compatiblity with blas
  #
  # 2. when using column-major layout, pressio4py references the
  #    matrix phi without doing a deep copy, which saves memory
  #    since a single jacobian matrix is alive.
  #
  phi = np.ones((10,3), order='F')

  # to create the linear decoder, one can simply do
  linearDecoder = rom.Decoder(phi)

  # linearDecoder exposes a method to evaluate the mapping
  fomState, romState = np.zeros(10), np.ones(3)
  linearDecoder.applyMapping(romState, fomState)
  print(fomState)

def rank2StateDecoder():
  # create the phi tensor
  # attention: we declare phi to be column-major for these reasons:
  #
  # 1. pressio4py uses blas (wherever possible) to operate on numpy arrays,
  #    so a column-major layout implies seamless compatiblity with blas
  #
  # 2. when using column-major layout, pressio4py references the
  #    matrix phi without doing a deep copy, which saves memory
  #    since a single jacobian matrix is alive.
  #
  # suppose that:
  # N = 10 is total FOM deg of freedom
  # numFields = 4 (e.g. density, x-vel, y-vel, temperature)
  # and romSize = 3
  #
  # each slice phi[:,:,k] basically corresponds to the POD modes for the k-th field
  #
  N = 10
  romSize = 3
  numFields = 4
  phi = np.ones((N, romSize, numFields), order='F')

  # to create the linear decoder, one can simply do
  linearDecoder = rom.rank2state.MultiFieldDecoder(phi)

  # linearDecoder exposes a method to evaluate the mapping
  fomState = np.zeros((N, numFields), order='F')
  romState = np.ones((romSize, numFields), order='F')
  linearDecoder.applyMapping(romState, fomState)
  print(fomState)

if __name__ == "__main__":
  rank1StateDecoder()
  rank2StateDecoder()
