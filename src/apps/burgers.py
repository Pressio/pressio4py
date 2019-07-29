
import numpy as np

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
