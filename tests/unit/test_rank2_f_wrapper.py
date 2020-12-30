
import pytest, math
import numpy as np
import test_rank2_f_wrapper_module as m

def testCnstr0():
  print("testCnstr0")
  a = m.construct0(5,6)
  assert(a.shape[0] == 5)
  assert(a.shape[1] == 6)
  assert(a.flags['F_CONTIGUOUS'])

def testCnstr1():
  print("testCnstr1")
  a = np.zeros((5,6), order='F')
  a_add = a.__array_interface__['data'][0]
  print("a: ", hex(a_add))
  assert(m.construct1(a, a_add) == True)

def testCnstr2():
  '''
  this file is for col-major tensor wrapper
  and this test is supposed to test the constructor
  with view semantics
  if we pass a col-major array it will be viewed
  if we pass a row-major array it will be copied
  '''
  print("testCnstr2")
  a = np.zeros((5,6), order='F')
  a_add = a.__array_interface__['data'][0]
  print("a: ", hex(a_add))
  assert(m.construct2(a, a_add) == True)
  a1 = np.zeros((5,6), order='C')
  a1_add = a1.__array_interface__['data'][0]
  print("a1: ", hex(a1_add))
  assert(m.construct2(a1, a1_add) == True)

def testCopyConstr():
  print("testCopyConstr")
  assert(m.copyConstruct() == True)

def testMoveConstr():
  print("testMoveConstr")
  a = np.zeros((5,6), order='F')
  a_add = a.__array_interface__['data'][0]
  print("a: ", hex(a_add))
  assert(m.moveCstr(a, a_add) == True)

def testExtent():
  print("testExtent")
  a = np.zeros((5,6))
  assert(m.extent(a))

def testSubscripting():
  print("testSubscripting")
  assert(m.subscript() == True)

def testSliceContiguousWithCorrectLayout():
  '''
  create numpy array
  pass a contiguous slice to cpp
  modify slice on cpp slice with some given numbers
  python origianl object should change
  '''
  print("testSliceContiguousWithCorrectLayout")
  a = np.zeros((5,6), order='F')
  # since this is colmajor, a cont slice is something that
  # takes e.g. a subset of columns
  m.sliceContiguousWithCorrectLayout(a[:,2:4])
  assert( np.allclose(a[:,2], np.ones(5)*2.) )
  assert( np.allclose(a[:,3], np.ones(5)*3.) )

# def testSliceNonContiguous():
#   '''
#   create numpy array
#   pass a non contiguous slice to cpp
#   (since it is noncont, the cpp side has to make
#   a copy because a numpy::array is contiguous by definition
#   so it cannot "view" a contiguous array so it has to make a copy)

#   modify slice on cpp slice
#   python origianl object should NOT change
#   '''
#   print("testSliceNonContiguous")
#   a = np.zeros(8)
#   b = a[2:8:2]
#   assert(b.shape[0] == 3)

#   b_add = b.__array_interface__['data'][0]
#   m.sliceNonContiguous(b, b_add)
#   gold = np.zeros(8)
#   assert( np.allclose(a, gold) )
