
import numpy as np

class Ops:
  # todo: fix names and mutability issues
  def __init__(self):
    pass

  def scale(self, a, coeff):
    # NOTE: we have to use *= or it won't change a in place
    a *= coeff

  def multiply1(self, a, b, transposeA=False):
    if transposeA == True:
      return np.dot(a.T, b)
    else:
      return np.dot(a, b)

  def multiply2(self, a, b, c, transposeA=False):
    if transposeA == True:
      c[:] = np.dot(a.T, b)
    else:
      c[:] = np.dot(a, b)

  def time_discrete_euler(self, R, yn, ynm1, dt):
    R[:] = yn[:] - ynm1[:] -dt*R[:]

  def time_discrete_jacobian(self, jphi, phi, factor, dt):
    jphi[:] = phi[:] -factor*dt*jphi[:]

  def myprint(self, A):
    print (np.atleast_2d(A[:]).T)
    #print(A)



class LinSolver:
  def __init__(self):
    pass

  def solve(self,A,b,x):
    x[:] = np.linalg.solve(A,b)
    #print('X address: {}'.format(hex(x.__array_interface__['data'][0])))
