
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

  def velocity(self, *args):
    u, t = args[0], args[1]
    if len(args) == 2:
      f = np.zeros(self.Ncell_)
      self.velocityImpl(u, t, f)
      return f
    else:
      f = args[2]
      self.velocityImpl(u, t, f)

  def velocityImpl(self, u, t, f):
    f[0] = 0.5 * self.dxInv_ * (self.mu_[0]**2 - u[0]**2)
    f[1:] = 0.5 * self.dxInv_ * (u[0:-1]**2 - u[1:]**2)
    f[:] += self.mu_[1] * np.exp( self.mu_[2] * self.xGrid_[:] )
    #for i in range(1, self.Ncell_):
    #  f[i] = 0.5 * self.dxInv_ * (u[i-1]**2 - u[i]**2)
    #for i in range(0, self.Ncell_):
    #  f[i] += self.mu_[1] * np.exp( self.mu_[2] * self.xGrid_[i] )

  def jacobianImpl(self, u, t, J):
    J[0][0] = -self.dxInv_*u[0]
    for i in range(1, self.Ncell_):
      J[i][i-1] = self.dxInv_ * u[i-1]
      J[i][i] = -self.dxInv_ * u[i]

  def jacobian(self, *args):
    u, t = args[0], args[1]
    if len(args) == 2:
      J = np.zeros((self.Ncell_, self.Ncell_))
      self.jacobianImpl(u, t, J)
      return J
    else:
      J = args[2]
      self.jacobianImpl(u, t, J)

  def applyJacobian(self, *args):
    # A = J*B, with J evaluated for given x,t
    u, B, t = args[0], args[1], args[2]
    if len(args) == 3:
      J = self.jacobian(u, t)
      return np.dot(J, B)
    else:
      A = args[3]
      J = self.jacobian(u, t)
      A[:] = np.dot(J, B)
