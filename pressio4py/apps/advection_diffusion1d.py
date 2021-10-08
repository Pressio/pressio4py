
import numpy as np
import math
from scipy.sparse import csr_matrix, diags, spdiags
from scipy import linalg
import time

# def lindiff(u,x=None,dudx=None):
#   '''
#   So far only works with k(x) dependence
#   '''
#   k = 1. + 0.*u
#   dkdx = 0. + 0.*u
#   return k, dkdx

def nonlindiff(u,x=None,dudx=None):
  pw = 4.
  k = x**pw
  dkdx = pw*(x**(pw-1))
  return k,dkdx

class AdvDiff1d():
  '''
  du/dt = d/dx( k(u,x) du/dx ) - a*du/dx

  where the RHS is f and it will be approx by FD:

  d2u/dU2 = (u_{i+1} - 2u_{i} + u_{i-1})/dx^2
  du/dx = (u_{i} - u_{i-1})/dx

  with Dirichlet i.e. fixed (zero) boundary conditions

  For linear case, k(u,x) = 1
  For nonlinear case, k(u,x) = 1 - u^2
  (1-u^2)d2u/dx2 + du/dx(d/dx(1-u^2))
  = (1-u^2)*d2u/dx2 -2u*(du/dx)^2
  '''

  def __init__(self,
               kfun=nonlindiff,
               nGrid=50,
               xL=0.0, xR=1.0,
               IC=None,
               adv_coef=1.0):
    self.xL = xL
    self.xR = xR
    self.nGrid = nGrid+2 # include boundary points
    if IC==None:
      self.ic = lambda x: 2.*np.sin(9.*np.pi*x) - np.sin(4.*np.pi*x)
    else:
      self.ic = IC

    self.kfun = kfun
    self.adv_coef = adv_coef
    self.setup()

  def setup(self):
    self.xGrid = np.linspace(self.xL,self.xR,self.nGrid)
    self.dx = np.diff(self.xGrid)[0]

    if self.ic is None:
      self.ic = np.sin(np.pi*self.xGrid)
      self.u0 = self.ic.copy()
    else:
      self.u0 = self.ic(self.xGrid)

    # create 2nd order diff matrix
    self.diff_mtx = np.diag(-2.0*np.ones(self.nGrid), 0) + \
      np.diag(+1.0*np.ones(self.nGrid-1),-1) + \
      np.diag(+1.0*np.ones(self.nGrid-1),+1)
    self.diff_mtx[0] = np.zeros(self.nGrid)
    self.diff_mtx[-1] = np.zeros(self.nGrid)

    # create 1st order advection FD matrix
    self.adv_mtx = np.diag(1.0*np.ones(self.nGrid), 0) + \
      np.diag(-1.0*np.ones(self.nGrid-1), -1)
    self.adv_mtx[0] = np.zeros(self.nGrid)
    self.adv_mtx[-1] = np.zeros(self.nGrid)

  def createVelocity(self):
    return np.zeros(self.nGrid)

  def velocity(self, u, t, f):
    x = self.xGrid
    dx = self.dx
    A = self.diff_mtx
    d2udx2 = (1.0/dx**2)*np.dot(A,u)
    B = self.adv_mtx
    dudx = (1.0/dx)*np.dot(B,u)
    k, dkdx = self.kfun(u,x=x,dudx=dudx)
    a = self.adv_coef
    f[:] = k*d2udx2 + dudx*dkdx - a*dudx
    # enforce fixed boundary conditions
    f[0] = 0.
    f[-1] = 0.

  def createApplyJacobianResult(self, B):
    return np.zeros_like(B)

  def applyJacobian(self, u, B, t, result):
    J = self.jacobian(u, t)
    result[:] = np.dot(J,B)

  def jacobian(self, u, t):
    x = self.xGrid
    dx = self.dx
    A = self.diff_mtx
    B = self.adv_mtx
    a = self.adv_coef
    k, dkdx = self.kfun(u,x=x)
    J = np.dot(np.diag(k),(1./dx**2)*A) + \
      (1./dx)*dkdx*B + \
      - a*(1./dx)*B

    return J
