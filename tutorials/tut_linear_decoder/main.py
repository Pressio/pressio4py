
import numpy as np
import random

# if run from within a build of pressio4py, need to append to python path
import pathlib, sys
file_path = pathlib.Path(__file__).parent.absolute()
sys.path.append(str(file_path) + "/../..")      # to access pressio4py lib

from pressio4py import rom as rom, logger

if __name__ == "__main__":
  logger.initialize(logger.logto.terminal, "null")
  logger.setVerbosity([logger.loglevel.info])

  '''
  A linear decoder (or mapping) represents

        y = phi * x

  where phi is the mapping's Jacobian (assumed constant for now).
  This is a typical approximation adopted in projection-based ROMs,
  where "x" is referred to as the generalized coordinates (or latent space)
  and "y" represents an approximation of the FOM state.

  This tutorial shows how to create a linear decoder/mapper in pressio4py.
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
  # Note: you can also use a row-major layout, but that forces
  # pressio4py to make a deep copy of matrix.

  phi = np.ones((10,3), order='F')

  # to create the linear decoder, one can simply do
  linearDecoder = rom.Decoder(phi)

  # linearDecoder exposes a method to evaluate the mapping
  y,x = np.zeros(10), np.ones(3)
  linearDecoder.applyMapping(x, y)

  print(y)
