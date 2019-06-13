
import pyrom
import numpy as np

np.set_printoptions(suppress=True,linewidth=np.nan,threshold=np.nan)

#-------------------------------------------------------
class Burgers1d:
  def __init__(self, params, Ncell):
    self.mu_ = params
    self.xL_ = 0.0
    self.xR_ = 100.
    self.Ncell_ = Ncell
    self.dx_ = 0.
    self.dxInv_ = 0.
    self.xGrid_ = np.zeros(self.Ncell_)
    self.U0_ = np.zeros(self.Ncell_)

  def setup(self):
    self.dx_ = (self.xR_ - self.xL_)/float(self.Ncell_)
    self.dxInv_ = 1.0/self.dx_;
    self.U0_[:] = 1.
    for i in range(0, self.Ncell_):
      self.xGrid_[i] = self.dx_*i + self.dx_*0.5

  def getInitialState(self):
    return self.U0_

  def residual1(self, yIn, t):
    r = np.zeros(self.Ncell_)
    self.residual2(yIn, r, t)
    return r

  def residual2(self, yIn, R, t):
    #print('Py res(Y) address: {}'.format(hex(yIn.__array_interface__['data'][0])))
    R[0] = 0.5 * self.dxInv_ * (self.mu_[0]**2 - yIn[0]**2)
    R[1:] = 0.5 * self.dxInv_ * (yIn[0:-1]**2 - yIn[1:]**2)
    R[:] += self.mu_[1] * np.exp( self.mu_[2] * self.xGrid_[:] )
    #for i in range(1, self.Ncell_):
    #  R[i] = 0.5 * self.dxInv_ * (yIn[i-1]**2 - yIn[i]**2)
    #for i in range(0, self.Ncell_):
    #  R[i] += self.mu_[1] * np.exp( self.mu_[2] * self.xGrid_[i] )

  def jacobian2(self, yIn, J, t):
    J[0][0] = -self.dxInv_*yIn[0]
    for i in range(1, self.Ncell_):
      J[i][i-1] = self.dxInv_ * yIn[i-1]
      J[i][i] = -self.dxInv_ * yIn[i]

  def jacobian1(self, yIn, t):
    J = np.zeros((self.Ncell_, self.Ncell_))
    self.jacobian2(yIn, J, t)
    return J

  # eval Jac and update in place A = Jac * B
  def applyJacobian2(self, yIn, B, A, t):
    J = self.jacobian1(yIn, t)
    A[:] = np.dot(J, B)

  # eval Jac and return Jac * B
  def applyJacobian1(self, yIn, B, t):
    J = self.jacobian1(yIn, t)
    return np.dot(J, B)

#-------------------------------------------------------
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
    print("factor", factor, " dt ", dt)
    jphi[:] = phi[:] -factor*dt*jphi[:]

  def myprint(self, A):
    print(A)

#-------------------------------------------------------
class LinSolver:
  def __init__(self):
    pass

  def solve(self,A,b,x):
    x[:] = np.linalg.solve(A,b)
    #print('X address: {}'.format(hex(x.__array_interface__['data'][0])))
    #print('X1 address: {}'.format(hex(x1.__array_interface__['data'][0])))


#-------------------------------------------------------
np.set_printoptions(precision=15)
Ncell = 20
mu = np.array([5., 0.02, 0.02])
pyApp = Burgers1d(mu,Ncell)
pyApp.setup()

yRef = pyApp.getInitialState()
#print(yRef)
#print('PY: yRef addr: {}'.format(hex(yRef.__array_interface__['data'][0])))

# object in charge of ops
ops = Ops()

# load basis into numpy array
phi = np.loadtxt("basis.txt")
#print('PY: phi addr: {}'.format(hex(yRef.__array_interface__['data'][0])))

# create a decoder
decoder = pyrom.LinearDecoder(phi, ops)

# the LSPG (reduced) state
romSize = phi.shape[1]
yRom = np.zeros(romSize)
#print('PY: yRom addr: {}'.format(hex(yRom.__array_interface__['data'][0])))

# the problem
t0 = 0.
lspgObj = pyrom.LspgProblem(pyApp, yRef, decoder, yRom, t0, ops)

# get stepper
stepper = lspgObj.getStepper()

# linear solver
lsO = LinSolver()

# non linear solver
nlsO = pyrom.GaussNewton(stepper, yRom, lsO, ops)
print( nlsO.getMaxIterations() )
print( nlsO.getTolerance() )
nlsO.setMaxIterations(20)
nlsO.setTolerance(1e-13)

dt = 0.01
pyrom.integrateNSteps(stepper, yRom, 0.0, dt, 10, nlsO)

















# print("y", y)
# r0 = pyApp.residual1(y, 0.)
# print("r0", r0)
# j0 = pyApp.jacobian1(y, 0.)
# print(j0)

#stepper = pyode_expl.ExplicitEuler(y, pyApp)
#dt = 0.01
#pyode_expl.integrateNSteps(stepper, y, 0.0, dt, 3500)
#print("y final", y)



# # if created from list DOES not work
# #y = np.array([1,2,3])
# # it needs to be creaed as zeros
# y = np.zeros(3)
# y[0] = 1.; y[1] = 2.; y[2] = 3.;

# print('Py Y address: {}'.format(hex(y.__array_interface__['data'][0])))
# pyode_expl.integrateNSteps(stepper, y, 0.0, dt, 3)
# print(y)

# y2 = np.zeros(3)
# y2[0] = 1.; y2[1] = 2.; y2[2] = 3.;
# stepper = pyode_expl.RungeKutta4(y2, pyApp)
# pyode_expl.integrateNSteps(stepper, y2, 0.0, dt, 1)
# print(y2)
