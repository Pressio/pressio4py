
import pyode_expl
import numpy as np

class MyFom:
  def __init__(self, params):
    self.params = params

  def residual1(self, yIn, t):
    return np.zeros(3)

  def residual2(self, yIn, R, t):
    print('Py res(Y) address: {}'.format(hex(yIn.__array_interface__['data'][0])))
    print("yIn", yIn, yIn.dtype)
    R[0] = yIn[0]
    R[1] = yIn[1]
    R[2] = yIn[2]
    print(R)

  def printFR(self, a):
    print(a)

  def update(self, v1, c1, v2, c2):
    for i in range(0,3):
      v1[i] = c1*v1[i] + c2*v2[i]
    print("----")


pyApp = MyFom("dummy")

dt = 0.1

# if created from list DOES not work
#y = np.array([1,2,3])
# it needs to be creaed as zeros
y = np.zeros(3)
y[0] = 1.; y[1] = 2.; y[2] = 3.;

print('Py Y address: {}'.format(hex(y.__array_interface__['data'][0])))

stepper = pyode_expl.ExplicitEuler(y, pyApp)
pyode_expl.integrateNSteps(stepper, y, 0.0, dt, 3)
print(y)

y2 = np.zeros(3)
y2[0] = 1.; y2[1] = 2.; y2[2] = 3.;
stepper = pyode_expl.RungeKutta4(y2, pyApp)
pyode_expl.integrateNSteps(stepper, y2, 0.0, dt, 1)
print(y2)









#main.test(appObj)
#print(a)
#main.foo(np.ones(3))
# s = np.zeros(5)
# s[:] = 1
# print(s)

# appObj = MyFom("dummy")
# r1 = appObj.residual(s, 0)
# print(r1)
