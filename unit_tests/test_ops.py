
import pytest
import numpy as np
import test_ops_module as m

def testDeepCopy():
  a = np.ones(5)
  a *= 2.
  b = m.deepCopyTest(a)
  assert b[0]==2
  assert b[1]==2
  assert b[2]==2
  assert b[3]==2
  assert b[4]==2

def testNorm1():
  a = np.ones(5)
  a *= 2.
  assert m.norm1Test(a) == 10.

def testNorm2():
  a = np.ones(5)
  a *= 2.
  assert m.norm2Test(a) == np.sqrt(20.)

def testDot():
  a = np.ones(5)
  a *= 2.
  b = np.ones(5)
  b *= 2.
  assert m.dotTest(a,b) == 20.

def testFill():
  a = np.ones(5)
  b = m.fillTest(a)
  assert b[0]==5.5
  assert b[1]==5.5
  assert b[2]==5.5
  assert b[3]==5.5
  assert b[4]==5.5

def testScale():
  a = np.ones(5)
  b = m.scaleTest(a)
  assert b[0]==2.
  assert b[1]==2.
  assert b[2]==2.
  assert b[3]==2.
  assert b[4]==2.

def testElemWiseMult():
  y = np.ones(5); y*=2;
  x = np.ones(5); x*=3;
  z = np.ones(5); z*=4;
  b = m.ewmultTest(y,x,z)
  for i in range(5):
    assert b[i]==y[i] + x[i]*z[i]

def testVectorUpdate1():
  v  = np.ones(5); v*=2;
  v1 = np.ones(5); v1*=3;
  v2 = np.ones(5); v2*=4;
  b = m.vecUpdTest(v,v1,v2)
  print(b)
  for i in range(5): assert b[0]==18

# def testprodRank3ARank2BRank2C():
#   assert(m.prodR3AR2BR2C())
