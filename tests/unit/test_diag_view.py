
import pytest
import numpy as np
import test_diag_view_module as m

def test1():
  A = np.zeros((4,4))
  A[0,0]=1.;
  A[1,1]=2.;
  A[2,2]=3.;
  A[3,3]=4.;

  a_add = A.__array_interface__['data'][0]
  print("A: ", hex(a_add))
  assert m.diagViewConstruct(A)

def test2():
  A = np.zeros((4,4))
  A[0,0]=1.;
  A[1,1]=2.;
  A[2,2]=3.;
  A[3,3]=4.;
  assert m.diagViewModify(A)
