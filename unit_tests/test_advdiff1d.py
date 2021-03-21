
import pytest
import numpy as np
import math

from pressio4py.apps.advection_diffusion1d import AdvDiff1d

def test1():
  fomObj = AdvDiff1d(nGrid=4, adv_coef=2.0)
  x = fomObj.xGrid
  print(x)
  assert(math.isclose(x[0], 0.))
  assert(math.isclose(fomObj.adv_coef, 2.))

# todo: add more
