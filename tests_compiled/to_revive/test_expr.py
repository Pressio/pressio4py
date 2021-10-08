
import pytest
import numpy as np
import test_expr_module as m

def test_span():
  print("\ntesting span")
  A = np.arange(8)
  #a_add = A.__array_interface__['data'][0]
  #print("A: ", hex(a_add))
  assert m.span(A)

def test_subspan():
  print("testing subspan")
  A = np.zeros((6,4), order='F')
  A[0,0] = 1.;  A[0,1] = 2.;  A[0,2] = 3.;  A[0,3] = 4.;
  A[1,0] = 5.;  A[1,1] = 6.;  A[1,2] = 7.;  A[1,3] = 8.;
  A[2,0] = 9.;  A[2,1] = 10.; A[2,2] = 11.; A[2,3] = 12.;
  A[3,0] = 13.; A[3,1] = 14.; A[3,2] = 15.; A[3,3] = 16.;
  A[4,0] = 17.; A[4,1] = 18.; A[4,2] = 19.; A[4,3] = 20.;
  A[5,0] = 21.; A[5,1] = 22.; A[5,2] = 23.; A[5,3] = 24.;
  assert m.subspan1(A)

  A = np.zeros((6,4), order='F')
  A[0,0] = 1.;  A[0,1] = 2.;  A[0,2] = 3.;  A[0,3] = 4.;
  A[1,0] = 5.;  A[1,1] = 6.;  A[1,2] = 7.;  A[1,3] = 8.;
  A[2,0] = 9.;  A[2,1] = 10.; A[2,2] = 11.; A[2,3] = 12.;
  A[3,0] = 13.; A[3,1] = 14.; A[3,2] = 15.; A[3,3] = 16.;
  A[4,0] = 17.; A[4,1] = 18.; A[4,2] = 19.; A[4,3] = 20.;
  A[5,0] = 21.; A[5,1] = 22.; A[5,2] = 23.; A[5,3] = 24.;
  assert m.subspan2(A)

def test_diag():
  print("testing diag")
  A = np.zeros((4,4), order='F')
  A[0,0] = 1.;  A[0,1] = 2.;  A[0,2] = 3.;  A[0,3] = 4.;
  A[1,0] = 5.;  A[1,1] = 6.;  A[1,2] = 7.;  A[1,3] = 8.;
  A[2,0] = 9.;  A[2,1] = 10.; A[2,2] = 11.; A[2,3] = 12.;
  A[3,0] = 13.; A[3,1] = 14.; A[3,2] = 15.; A[3,3] = 16.;
  assert m.diag1(A)

  A = np.zeros((4,4), order='F')
  A[0,0] = 1.;  A[0,1] = 2.;  A[0,2] = 3.;  A[0,3] = 4.;
  A[1,0] = 5.;  A[1,1] = 6.;  A[1,2] = 7.;  A[1,3] = 8.;
  A[2,0] = 9.;  A[2,1] = 10.; A[2,2] = 11.; A[2,3] = 12.;
  A[3,0] = 13.; A[3,1] = 14.; A[3,2] = 15.; A[3,3] = 16.;
  assert m.diag2(A)
