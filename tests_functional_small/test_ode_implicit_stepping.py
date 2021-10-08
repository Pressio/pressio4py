
import numpy as np
from pressio4py import solvers, ode

class MySys:
  def __init__(self):
    pass

  def createVelocity(self):
    return np.zeros(5)

  def createJacobian(self):
    return np.zeros((5,5))

  def velocity(self, stateIn, time, f):
    f[:] = 3.0

  def jacobian(self, stateIn, time, J):
    print(J.shape)
    J[:] = 1.

def test_bdf1_constructor():
  print("\n")
  state = np.ones(5)
  sys = MySys()
  scheme = ode.stepscheme.BDF1
  stepper = ode.create_implicit_stepper(scheme, state, sys)
  print(stepper.order())


#----------------------------
class MyLinSolver:
  def __init__(self):
    self.callCount_ = 0

  def solve(self, A,b,x):
    print("\n Python Lin solver")
    print(A)
    print(b)
    print(x)
    self.callCount_ += 1
    x[:] += 1.

#----------------------------
class MyNonLinSolver:
  def __init__(self, system):
    self.R_ = system.createResidual()
    self.J_ = system.createJacobian()

  def solve(self, system, x):
    print("\n Python NonLin solver")
    system.residual(x, self.R_)
    system.jacobian(x, self.J_)
    print(x)
    print(self.R_)
    print(self.J_)


def test_bdf1_call():
  print("\n")
  state = np.ones(5)
  sys = MySys()
  scheme = ode.stepscheme.BDF1
  stepper = ode.create_implicit_stepper(scheme, state, sys)
  print("stepper address = ", hex(id(stepper)))

  # linear and non linear solver
  lsO = MyLinSolver()
  print("lsO address = ", hex(id(lsO)))
  nlsO = solvers.create_newton_raphson(stepper, state, lsO)

  # # linear and non linear solver
  # lsO = MyLinSolver()
  # nlsO = solvers.create_newton_raphson(stepper, state, lsO)
  # nlsO.setUpdatingCriterion(solvers.update.Standard)
  # nlsO.setMaxIterations(2)
  # nlsO.setStoppingCriterion(solvers.stop.AfterMaxIters)

  # #mynlsO = MyNonLinSolver(stepper)
  # #stepper(state, 0., 1.2, 1, mynlsO)

  # ode.advance_n_steps(stepper, state, 0., 1.2, 3, nlsO)

  # print("before step: ", state)
  # assert( np.all(state == 1.) )
  # stepper(state, 0., 2., 1)
  # print("after step: ", state)
  # '''
  # state = state + dt * f
  # where f is MySys velocity
  # so we should have:
  # '''
  # assert( np.all(state == 7.) )

# def test_forward_euler_advance_n_steps():
#   print("test_forward_euler_advance_n_steps")
#   state = np.ones(5)
#   sys = MySys()
#   scheme = ode.stepscheme.ForwardEuler
#   stepper = ode.create_explicit_stepper(scheme, state, sys)
#   dt = 1.2
#   dtcb = MyDtSetter()
#   ode.advance_n_steps(stepper, state, 0., dtcb, 1)
#   print("after step: ", state)
#   assert( np.all(state == 4.6) )

# def test_forward_euler_advance_n_steps_and_observe():
#   print("test_forward_euler_advance_n_steps_and_observe")
#   state = np.ones(5)
#   sys = MySys()
#   scheme = ode.stepscheme.ForwardEuler
#   stepper = ode.create_explicit_stepper(scheme, state, sys)
#   dt = 1.2

#   class MyObs:
#     def __init__(self):
#       pass
#     def __call__(self, step, time, state):
#       print("I got cal")
#       print(state)
#       assert(step == 0 or step == 1)
#       if (step ==0):
#         assert( np.all(state == 1.) )
#       if (step ==1):
#         assert( np.all(state == 4.6) )

#   ode.advance_n_steps_and_observe(stepper,state,0.,dt,1, MyObs())
#   print("after obs: ", state)


#if __name__ == '__main__':
#test_forward_euler()
