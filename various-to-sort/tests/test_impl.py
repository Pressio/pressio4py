
import pyode_impl
import example
import numpy as np

# again, here, make sure you use [] operator for the matter or
# mutability/immutability and local scoping

class MyFom:
  def __init__(self, params):
    self.params = params

  def update(self, v1, c1, v2, c2):
    v1[:] = c1*v1[:] + c2*v2[:]
    print("----")

  def residual2(self, yIn, R, t):
    print('Py res(Y) address: {}'.format(hex(yIn.__array_interface__['data'][0])))
    print("yIn", yIn, yIn.dtype)
    R[:] = -10.*yIn[:]
    print(R)

  def jacobian2(self, yIn, J, t):
    print("yIn", yIn)
    J[0][0] = -10.;
    J[1][1] = -10.;
    J[2][2] = -10.;

  def residual1(self, yIn, t):
    return np.zeros(3)

  def jacobian1(self, yIn, t):
    return np.zeros((3,3))


class LinSolver:
  def __init__(self, params):
    self.params = params

  def solve(self,A,b,x):
    print('X address: {}'.format(hex(x.__array_interface__['data'][0])))
    print("A", A)
    print("b",b)
    x1 = np.linalg.solve(A,b)
    print('X1 address: {}'.format(hex(x1.__array_interface__['data'][0])))
    # again, here, make sure you use [] operator because doing x=X1
    # implies data remains local, the underlying x data is not really modified
    x[:] = x1[:]


class Ops:
  # todo: fix names and mutability issues
  def __init__(self, params):
    self.params = params

  def multiply2(self, a, b, c):
    c[:] = a[:] * b[:]

  def multiply1(self, a, b):
    return a[:] * b[:]

  def multiply_transpose2(self, a, b, c):
    print("a.shape", a.shape)
    print("b.shape", b.shape)
    print("c.shape", c.shape)
    c = a.T[:] * b

  def self_multiply_transpose(self, a, c):
    print("se a.shape", a.shape)
    print("se c.shape", c.shape)
    c[:] = a.T * a
    print ("self done")

  def multiply_transpose1(self, a, b):
    print ("a", a)
    print ("b", b)
    print ("a.T", a.T)
    return a.T * b

  def scale(self, a, coeff):
    a[:] *= coeff

###############################
###############################

dt = 0.01
pyApp = MyFom("dummy")
y = np.zeros(3)
y[0] = 1.; y[1] = 2.; y[2] = 3.;
print('Py Y address: {}'.format(hex(y.__array_interface__['data'][0])))

# stepper
stepper = pyode_impl.ImplicitEuler(y, pyApp)

# linear solver
lsO = LinSolver("somename or smelse")

# object knwoing how to do operations
ops = Ops("myOps")

# non linear solver
nlsO = pyode_impl.GaussNewton(stepper, y, lsO, ops)
nlsO.setMaxIterations(2)

pyode_impl.integrateNSteps(stepper, y, 0.0, dt, 1, nlsO)
print(y)





# class Obj:
#   def __init__(self):
#     pass

#   def run2(self, a, b):
#     print (a)
#     d = np.ones(3)
#     a[:] = d
#     print (a)

# gg = Obj()
# TC = example.TestClass(gg)
# TC.run()

# a = np.ones((3,3))
# a[0][0] = 2.
# a[1][1] = 3.
# a[2][2] = 4.

# b = np.ones((3,3))
# c = a + b
# c1 = example.add_arrays(a,b)
# print (c)
# print("---")
# print (c1)


# nonlinear solver
#nlsO = pysolver.NewtonRaphson(lsO)
#nlsO.setMaxIterations(15)
#nlsO.solve(pyApp, y)




#stepper(y, 0, 0, 1, solver)
#print(y)

# using stepper_t = ode::ImplicitStepper<
# ode::ImplicitEnum::Euler,
# state_t, res_t, jac_t, app_t>; /*aux stepper NOT needed for backEuler*/
# stepper_t stepperObj(y, appObj);

# // define solver
# using lin_solver_t = solvers::iterative::EigenIterative<
# solvers::linear::iterative::Bicgstab, jac_t>;
# solvers::NewtonRaphson<double, lin_solver_t> solverO;

# // integrate in time
# int nSteps = 2;
# double dt = 0.01;
# ode::integrateNSteps(stepperObj, y, 0.0, dt, nSteps, solverO);
# std::cout << std::setprecision(14) << *y.data() << "\n";





# pyode_expl.integrateNSteps(stepper, y, 0.0, dt, 3)
# print(y)

# y2 = np.zeros(3)
# y2[0] = 1.; y2[1] = 2.; y2[2] = 3.;
# stepper = pyode_expl.RungeKutta4(y2, pyApp)
# pyode_expl.integrateNSteps(stepper, y2, 0.0, dt, 1)
# print(y2)









#main.test(appObj)
#print(a)
#main.foo(np.ones(3))
# s = np.zeros(5)
# s[:] = 1
# print(s)

# appObj = MyFom("dummy")
# r1 = appObj.residual(s, 0)
# print(r1)
