
import pytest
import numpy as np
import test_ops_rank1_array_module as m

def test_extent():
  print("\n")
  print("testing extent")
  a = np.ones(5)
  assert(m.extent(a))

def test_clone():
  print("\n")
  print("testing clone")
  a = np.array([1,2,3,4,5,6])
  b = m.clone(a)
  assert(b[0]==1.)
  assert(b[1]==2.)
  assert(b[2]==3.)
  assert(b[3]==4.)
  assert(b[4]==5.)
  assert(b[5]==6.)

def test_deep_copy():
  print("\n")
  print("testing deep_copy")
  a = np.ones(5)
  a *= 2.
  b = np.zeros(5)
  m.deep_copy(b,a)
  assert( np.all(b == 2.) )

def test_abs():
  print("\n")
  print("testing abs")
  a = -np.ones(5)
  b =  np.zeros(5)
  m.abs(b,a) # b = abs(a)
  assert( np.all(b == 1.) )

def test_scale():
  print("\n")
  print("testing scale")
  a = np.ones(5)*2.
  m.scale(a, 3.)
  assert(np.all(a == 6.))

def test_fill():
  print("\n")
  print("testing fill")
  a = np.ones(5)
  m.fill(a, 3.)
  assert(np.all(a == 3.))

def test_set_zero():
  print("\n")
  print("testing set_zero")
  a = np.ones(5)
  m.set_zero(a)
  assert(np.all(a == 0.))

def test_max_min():
  print("\n")
  print("testing max/min")
  a = np.array([-1,2,3.,4.,5.6,-4,-2,3])
  assert(m.max(a) == 5.6)
  assert(m.min(a) == -4.)

def test_norm():
  print("\n")
  print("testing norms")
  a = np.ones(5)
  a *= 2.
  assert m.norm1(a) == 10.

  a = np.ones(5)
  a *= 2.
  assert m.norm2(a) == np.sqrt(20.)

def test_dot():
  print("\n")
  print("testing dot")
  a = np.ones(5)*3.
  b = np.ones(5)*2.
  assert( m.dot(a,b) == 30.)




# def test_add_to_diagonal():
#   print("\n testing add_to_diagonal")
#   a = np.zeros((5,5))
#   m.add_to_diagonal(a)
#   for i in range(5):
#     for j in range(5):
#       if i==j:
#         assert(a[i,j]==5.5)
#       else:
#         assert(a[i,j]==0.0)

#   b = np.zeros((5,5),order='F')
#   m.add_to_diagonal(b)
#   for i in range(5):
#     for j in range(5):
#       if i==j:
#         assert(b[i,j]==5.5)
#       else:
#         assert(b[i,j]==0.0)


# def testDot():
#   a = np.ones(5)
#   a *= 2.
#   b = np.ones(5)
#   b *= 2.
#   assert m.dotTest(a,b) == 20.

# def testElemWiseMult():
#   y = np.ones(5); y*=2;
#   x = np.ones(5); x*=3;
#   z = np.ones(5); z*=4;
#   b = m.ewmultTest(y,x,z)
#   for i in range(5):
#     assert b[i]==y[i] + x[i]*z[i]

# def testVectorUpdate1():
#   v  = np.ones(5); v*=2;
#   v1 = np.ones(5); v1*=3;
#   v2 = np.ones(5); v2*=4;
#   b = m.vecUpdTest(v,v1,v2)
#   print(b)
#   for i in range(5): assert b[0]==18

# def testprodRank3ARank2BRank2C():
#   assert(m.prodR3AR2BR2C())
