
import pytest
import numpy as np
import test_span_module as m

def test1():
  A = np.arange(8)
  a_add = A.__array_interface__['data'][0]
  print("A: ", hex(a_add))
  assert m.spanConstruct(A)

def test2():
  A = np.arange(8)
  assert m.spanModify(A)
