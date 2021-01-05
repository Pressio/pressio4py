
import numpy as np
import pathlib, sys

def doFom(fom, dt, nsteps, saveFreq):
  u = fom.u0.copy()
  U = [u]
  f = fom.createVelocity()
  for i in range(1,nsteps+1):
    # query rhs of discretized system
    fom.velocity(u, i*dt, f)
    # simple Euler forward
    u = u + dt*f
    if i % saveFreq == 0:
      U.append(u)
  Usolns = np.array(U)
  return [u, Usolns.T]
