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
from numba import jit

@jit(nopython=True)
def time_discrete_euler_numba(R, yn, ynm1, dt):
  n = len(R)
  for i in range(n):
    R[i] = yn[i] - ynm1[i] -dt*R[i]

@jit(nopython=True)
def time_discrete_jacobian_numba(jphi, phi, factor, dt):
  nr, nc = jphi.shape[0], jphi.shape[1]
  for i in range(nr):
    for j in range(nc):
      jphi[i][j] = phi[i][j] + factor*dt*jphi[i][j]

@jit(nopython=True)
def scale_numba(a, coeff):
  for i in range(len(a)):
    a[i] *= coeff

def multiply_1(a, transA, b, transB):
  if transA == True and transB == False:
    return np.matmul(a.T, b)
  elif transA == True and transB == True:
    return np.matmul(a.T, b.T)
  elif transA == False and transB == True:
    return np.matmul(a, b.T)
  else:
    return np.matmul(a, b)

# def multiply_2(a, transA, b, transB, c):
#   if transA == True and transB == False:
#     c[:] = np.dot(a.T, b)
#   elif transA == True and transB == True:
#     c[:] = np.dot(a.T, b.T)
#   elif transA == False and transB == True:
#     c[:] = np.dot(a, b.T)
#   else:
#     c[:] = np.dot(a, b)


##########################################
##########################################
class Ops:
  # todo: fix names and mutability issues
  def __init__(self):
    pass

  def scale(self, a, coeff):
    scale_numba(a, coeff)
    # NOTE: we have to use *= or it won't change a in place
    #a *= coeff

  # this is called with either one of these:
  # 1. a, transA, b, transB, result
  # 2. a, transA, b, transB  (here we need to return result)
  def multiply(self, *args):
    a, transA, b, transB = args[0],args[1],args[2],args[3]
    if len(args) == 4:
      return multiply_1(a, transA, b, transB)
    elif len(args) == 5:
      args[4][:] = multiply_1(a, transA, b, transB)
    else:
      return;

  def time_discrete_euler(self, R, yn, ynm1, dt):
    time_discrete_euler_numba(R, yn, ynm1, dt)

  def time_discrete_jacobian(self, jphi, phi, factor, dt):
    time_discrete_jacobian_numba(jphi, phi, factor, dt)

  def myprint(self, A):
    np.set_printoptions(precision=15)
    if A.ndim == 1: print (np.atleast_2d(A[:]).T)
    else: print (np.atleast_2d(A[:]))




# class LinSolver:
#   def __init__(self):
#     pass
#   def solve(self,A,b,x):
#     x[:] = np.linalg.solve(A,b)
#     #print('X address: {}'.format(hex(x.__array_interface__['data'][0])))


#   # def multiply1(self, a, b, transposeA=False):
#   #   if transposeA == True:
#   #     #return np.dot(a.T, b)
#   #     return np.matmul(a.T, b)
#   #   else:
#   #     #return np.dot(a, b)
#   #     return np.matmul(a, b)

#   # def multiply2(self, a, b, c, transposeA=False):
#   #   if transposeA == True:
#   #     #c[:] = np.dot(a.T, b)
#   #     c[:] = np.matmul(a.T, b)
#   #   else:
#   #     #c[:] = np.dot(a, b)
#   #     c[:] = np.matmul(a, b)
