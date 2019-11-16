# ************************************************************************
#
# pressio4pyOps.py
#                     		  Pressio
#                             Copyright 2019
#    National Technology & Engineering Solutions of Sandia, LLC (NTESS)
#
# Under the terms of Contract DE-NA0003525 with NTESS, the
# U.S. Government retains certain rights in this software.
#
# Pressio is licensed under BSD-3-Clause terms of use:
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
# IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Questions? Contact Francesco Rizzi (fnrizzi@sandia.gov)
#
# ************************************************************************
#

import numpy as np
from numba import njit

@njit(["void(float64[::1], float64[::1], float64[::1], f8)"], fastmath=True)
def time_discrete_euler_numba(R, yn, ynm1, dt):
  n = len(R)
  for i in range(n):
    R[i] = yn[i] - ynm1[i] -dt*R[i]

@njit(["void(float64[::1,:], float64[::1,:], f8, f8)"], fastmath=True)
def time_discrete_jacobian_numba(jphi, phi, factor, dt):
  nr, nc = jphi.shape[0], jphi.shape[1]
  for i in range(nr):
    for j in range(nc):
      jphi[i][j] = phi[i][j] + factor*dt*jphi[i][j]

##########################################
class Ops:
  # todo: fix names and mutability issues
  def __init__(self):
    pass

  @staticmethod
  def time_discrete_euler(R, yn, ynm1, dt):
    R[:] = yn[:] - ynm1[:] -dt*R[:]
    #time_discrete_euler_numba(R, yn, ynm1, dt)

  @staticmethod
  def time_discrete_jacobian(jphi, phi, factor, dt):
    jphi[:] = phi[:] + factor*dt*jphi[:]
    #time_discrete_jacobian_numba(jphi, phi, factor, dt)

  @staticmethod
  def myprint(A):
    np.set_printoptions(precision=15)
    if A.ndim == 1: print (np.atleast_2d(A[:]).T)
    else: print (np.atleast_2d(A[:]))
