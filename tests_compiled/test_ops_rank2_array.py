
import pytest
import numpy as np
import test_ops_rank2_array_module as m

def test_extent():
  print("\n")
  print("\ntesting extent")
  a = np.ones((5,6), order='C')
  assert(m.extent(a))
  a = np.ones((5,6), order='F')
  assert(m.extent(a))

def test_clone():
  print("\n")
  print("testing clone")
  a = np.ones((5,6), order='C')
  b = m.clone(a)
  assert( np.all(b == 1.) )

  a1 = np.ones((5,6), order='F')
  b1 = m.clone(a1)
  assert( np.all(b1 == 1.) )

def test_deep_copy():
  print("\n")
  print("testing deep_copy")
  a = np.ones((5,6), order='C') * 2.
  b = np.ones((5,6), order='F')
  m.deep_copy(b, a)
  assert( np.all(b == 2.) )

  a = np.ones((5,6), order='F') * 3.
  b = np.ones((5,6), order='F')
  m.deep_copy(b, a)
  assert( np.all(b == 3.) )

  a = np.ones((5,6), order='C') * 4.
  b = np.ones((5,6), order='C')
  m.deep_copy(b, a)
  assert( np.all(b == 4.) )

  a = np.ones((5,6), order='F') * 5.
  b = np.ones((5,6), order='C')
  m.deep_copy(b, a)
  assert( np.all(b == 5.) )

def test_scale():
  print("\n")
  print("testing scale")
  a = np.ones((6,5), order='C')*2.
  m.scale(a, 3.)
  assert(np.all(a == 6.))

  a = np.ones((6,5), order='F')*2.
  m.scale(a, 3.)
  assert( np.all(a == 6.))

def test_fill():
  print("\n")
  print("testing fill")
  a = np.ones((6,5), order='C')
  m.fill(a, 3.)
  assert(np.all(a == 3.))

  a = np.ones((6,5), order='F')
  m.fill(a, 3.)
  assert( np.all(a == 3.))

def test_set_zero():
  print("\n")
  print("testing set_zero")
  a = np.ones((6,5), order='C')
  m.set_zero(a)
  assert(np.all(a == 0.))

  a = np.ones((6,5), order='F')
  m.set_zero(a)
  assert( np.all(a == 0.))

def test_add_to_diagonal():
  print("\n")
  print("testing add_to_diagonal")
  a = np.ones((6,6), order='C')*2.
  m.add_to_diagonal(a, 3.)
  for i in range(6):
    for j in range(6):
      if (i==j):
        assert(a[i,j]==5.)
      else:
        assert(a[i,j]==2.)

  a = np.ones((6,6), order='F')*2.
  m.add_to_diagonal(a, 3.)
  for i in range(6):
    for j in range(6):
      if (i==j):
        assert(a[i,j]==5.)
      else:
        assert(a[i,j]==2.)
