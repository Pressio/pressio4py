
from pressio4py import rom as rom

if __name__ == "__main__":
  '''
  In the ROM context, a linear decoder (or mapping) represents

        y_fom = phi * y_rom

  where phi is the mapping's Jacobian (assumed constant for now).
  This is a typical approximation adopted in projection-based ROMs,
  where "y_rom" is referred to as the generalized coordinates (or latent space)
  and "y_fom" represents an approximation of the FOM state.

  For examples using it, see subsequent tutorials.
  '''

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
  print(y)
