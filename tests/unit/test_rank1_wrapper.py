
import pytest, math
import numpy as np
import test_rank1_wrapper_module as m

def testCnstr0():
  print("testCnstr0")
  a = m.construct0(5)
  assert(a.shape[0] == 5)

def testCnstr1():
  print("testCnstr1")
  a = np.zeros(5)
  a_add = a.__array_interface__['data'][0]
  print("a: ", hex(a_add))
  assert(m.construct1(a, a_add) == True)

def testCnstr2():
  print("testCnstr2")
  a = np.zeros(5)
  a_add = a.__array_interface__['data'][0]
  print("a: ", hex(a_add))
  assert(m.construct2(a, a_add) == True)

def testCopyConstr():
  print("testCopyConstr")
  assert(m.copyConstruct() == True)

def testMoveConstr():
  print("testMoveConstr")
  a = np.zeros(5)
  a_add = a.__array_interface__['data'][0]
  print("a: ", hex(a_add))
  assert(m.moveCstr(a, a_add) == True)

def testExtent():
  print("testExtent")
  a = np.zeros(5)
  size = m.extent(a)
  assert size == 5

def testSubscripting():
  print("testSubscripting")
  assert(m.subscript() == True)

def testSliceContiguous():
  '''
  create numpy array
  pass a contiguous slice to cpp
  modify slice on cpp slice
  python origianl object should change
  '''
  print("testSliceContiguous")
  a = np.zeros(5)
  m.sliceContiguous(a[2:4])
  assert( math.isclose(a[2], 1.1) )
  assert( math.isclose(a[3], 2.2) )

def testSliceNonContiguous():
  '''
  create numpy array
  pass a non contiguous slice to cpp
  (since it is noncont, the cpp side has to make
  a copy because a numpy::array is contiguous by definition
  so it cannot "view" a contiguous array so it has to make a copy)

  modify slice on cpp slice
  python origianl object should NOT change
  '''
  print("testSliceNonContiguous")
  a = np.zeros(8)
  b = a[2:8:2]
  assert(b.shape[0] == 3)

  b_add = b.__array_interface__['data'][0]
  m.sliceNonContiguous(b, b_add)
  gold = np.zeros(8)
  assert( np.allclose(a, gold) )
