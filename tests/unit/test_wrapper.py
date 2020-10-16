
import pytest
import numpy as np
import test_wrapper_module as m

class CallBackHelper:
  def __init__(self):
    pass

  def cb(self, a, cppAddress):
    a_add = a.__array_interface__['data'][0]
    print("a: ", a_add)
    assert(a_add==cppAddress)

############################################
def test1():
  a = np.zeros(5)
  a_add = a.__array_interface__['data'][0]
  print("a: ", hex(a_add))
  assert(m.constructWrapper1(a, a_add) == True)

def test2():
  a = np.zeros(5)
  a_add = a.__array_interface__['data'][0]
  print("a: ", hex(a_add))
  assert(m.constructWrapper2(a, a_add) == True)

def test3():
  a = np.zeros(5)
  a_add = a.__array_interface__['data'][0]
  print("a: ", hex(a_add))
  cb = CallBackHelper()
  m.constructWrapper3(a, cb)

def extent():
  a = np.zeros(5)
  size = m.extentTest(a)
  assert size == 5

def subscriptChange():
  a = np.zeros(5)
  a1 = m.subscriptChange(a)
  print(a1)
  assert a1[0] == 1.
  assert a1[1] == 0.
  assert a1[2] == 0.
  assert a1[3] == 0.
  assert a1[4] == 0.

def moveConstr():
  a = np.zeros(5)
  a_add = a.__array_interface__['data'][0]
  print("a: ", hex(a_add))
  assert(m.testMoveCstr(a, a_add) == True)
