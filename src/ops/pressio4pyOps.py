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
    jphi[:] = phi[:] -factor*dt*jphi[:]

  def myprint(self, A):
    print (np.atleast_2d(A[:]).T)
    #print(A)



class LinSolver:
  def __init__(self):
    pass

  def solve(self,A,b,x):
    x[:] = np.linalg.solve(A,b)
    #print('X address: {}'.format(hex(x.__array_interface__['data'][0])))
