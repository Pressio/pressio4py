
import pysolver
import numpy as np

class MyFom:
  def __init__(self, params):
    self.params = params

  def residual2(self, yIn, R):
    print('Py res(Y) address: {}'.format(hex(yIn.__array_interface__['data'][0])))
    print("yIn", yIn, yIn.dtype)
    R[0] =  yIn[0]*yIn[0]*yIn[0] + yIn[1] - 1.0;
    R[1] = -yIn[0] + yIn[1]*yIn[1]*yIn[1] + 1.0;
    print(R)

  def residual1(self, yIn):
    R = np.zeros(2)
    self.residual2(yIn, R)
    return R

  def jacobian2(self, yIn, J):
    print("yIn", yIn)
    J[0][0] = 3.0*yIn[0]*yIn[0];
    J[0][1] =  1.0;
    J[1][0] = -1.0;
    J[1][1] = 3.0*yIn[1]*yIn[1];
    print(J)

  def jacobian1(self, yIn):
    J = np.zeros((2,2))
    self.jacobian2(yIn, J)
    return J

class LinSolver:
  def __init__(self, params):
    self.params = params

  def solve(self,A,x,b):
    print('X address: {}'.format(hex(x.__array_interface__['data'][0])))
    print("A", A)
    print("b",b)
    x1 = np.linalg.solve(A,b)
    print('X1 address: {}'.format(hex(x1.__array_interface__['data'][0])))
    # again, here, make sure you use [] operator for the matter or
    # mutability/immutability and local scoping
    x[:] = x1[:]
    print("x",x)

pyApp = MyFom("dummy")

y = np.zeros(2)
print('Py Y address: {}'.format(hex(y.__array_interface__['data'][0])))

# linear solver
lsO = LinSolver("somename or smelse")

# nonlinear solver
nlsO = pysolver.NewtonRaphson(lsO)
nlsO.setMaxIterations(15)
nlsO.solve(pyApp, y)
