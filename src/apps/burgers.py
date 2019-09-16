/*
//@HEADER
// ************************************************************************
//
// burgers.py
//                     		  Pressio
//                             Copyright 2019
//    National Technology & Engineering Solutions of Sandia, LLC (NTESS)
//
// Under the terms of Contract DE-NA0003525 with NTESS, the
// U.S. Government retains certain rights in this software.
//
// Pressio is licensed under BSD-3-Clause terms of use:
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived
// from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
// IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact Francesco Rizzi (fnrizzi@sandia.gov)
//
// ************************************************************************
//@HEADER
*/

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
