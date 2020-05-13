
import numpy as np
import math
from numba import jit, njit
from scipy.sparse import csr_matrix, diags, spdiags
from scipy import linalg
import time

@njit(["void(float64[:], f8, float64[:], float64[:], f8, f8)"])
def velocityImplNumba(u, t, f, expVec, dxInvHalf, mu0):
  n = len(u)
  uSq = np.square(u)
  f[0] = dxInvHalf * (math.pow(mu0, 2) - uSq[0]) + expVec[0]
  for i in range(1,n):
    f[i] = dxInvHalf * ( uSq[i-1] - uSq[i] ) + expVec[i]

@njit(["void(float64[:], float64[:], float64[:], f8)"])
def fillDiag(u, diag, ldiag, dxInv):
  n = len(u)
  for i in range(n-1):
    diag[i] = -dxInv*u[i]
    ldiag[i] = dxInv*u[i]
  diag[n-1] = -dxInv*u[n-1]

class Burgers1dSparseJacobian:
  def __init__(self, Ncell):
    self.mu_    = np.array([5., 0.02, 0.02])
    self.xL_    = 0.
    self.xR_    = 100.
    self.Ncell_ = Ncell
    self.dx_    = 0.
    self.dxInv_ = 0.
    self.dxInvHalf_ = 0.
    self.xGrid_ = np.zeros(self.Ncell_)
    self.U0_    = np.zeros(self.Ncell_)
    self.f_     = np.zeros(self.Ncell_)
    self.expVec_= np.zeros(self.Ncell_)
    self.diag_  = np.zeros(self.Ncell_)
    self.ldiag_ = np.zeros(self.Ncell_-1)
    self.setup()

  def setup(self):
    self.dx_ = (self.xR_ - self.xL_)/float(self.Ncell_)
    self.dxInv_ = (1.0/self.dx_)
    self.dxInvHalf_ = 0.5 * self.dxInv_
    for i in range(0, self.Ncell_):
      self.U0_[i] = 1.
      self.xGrid_[i] = self.dx_*i + self.dx_*0.5
    self.expVec_ = self.mu_[1] * np.exp( self.mu_[2] * self.xGrid_ )

  def velocity(self, u, t):
    velocityImplNumba(u, t, self.f_, self.expVec_, self.dxInvHalf_, self.mu_[0])
    return self.f_

  def jacobian(self, u, t):
    fillDiag(u, self.diag_, self.ldiag_, self.dxInv_)
    return diags( [self.ldiag_, self.diag_], [-1,0], format='csr')

  def applyJacobian(self, u, B, t):
    J = self.jacobian(u, t)
    return J.dot(B)
