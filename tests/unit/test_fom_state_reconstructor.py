
import numpy as np
from pressio4py import rom as rom

def test_reconstructor():
  print("test_reconstructor")
  fomSize, romSize = 6, 3
  phi = np.array([[1, 1, 1],
                  [2, 2, 2],
                  [3, 3, 3],
                  [4, 4, 4],
                  [5, 5, 5],
                  [6, 6, 6]])

  yFom = np.ones(fomSize)
  decoder = rom.Decoder(phi)
  #decoder = rom.FomReconstructor(yFom, decoder)
  # yRom = np.array([1.,2.,3.])
  # decoder.applyMapping(yRom, yFom)
  # print(yFom)
  # assert(yFom[0]==6)
  # assert(yFom[1]==12)
  # assert(yFom[2]==18)
  # assert(yFom[3]==24)
  # assert(yFom[4]==30)
  # assert(yFom[5]==36)
