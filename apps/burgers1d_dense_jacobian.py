
import numpy as np
import math
from numba import jit, njit
from scipy.sparse import csr_matrix, diags, spdiags
from scipy import linalg
import time

@njit(["void(float64[::1], f8, float64[::1], float64[::1], f8, f8)"])
def velocityImplNumba(u, t, f, expVec, dxInvHalf, mu0):
  n = len(u)
  uSq = np.square(u)
  f[0] = dxInvHalf * (math.pow(mu0, 2) - uSq[0]) + expVec[0]
  for i in range(1,n):
    f[i] = dxInvHalf * ( uSq[i-1] - uSq[i] ) + expVec[i]


@njit(["void(float64[::1], f8, float64[::1, :], f8, int32)"])
def jacobianImplNumba(u, t, J, dxInv, N):
  J[N-1][N-1] = -dxInv*u[N-1]
  for i in range(0, N-1):
    J[i][i] = -dxInv * u[i]
    J[i+1][i] = dxInv * u[i]


class Burgers1dDenseJacobian:
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
    self.J_   = np.zeros((self.Ncell_, self.Ncell_), order='F')
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
    jacobianImplNumba(u, t, self.J_, self.dxInv_, self.Ncell_)
    return self.J_

  def applyJacobian(self, u, B, t):
    # we could call matmul here since J, B are dense, but calling blas
    # directly is more efficient
    jacobianImplNumba(u, t, self.J_, self.dxInv_, self.Ncell_)
    return linalg.blas.dgemm(1., self.J_, B)
