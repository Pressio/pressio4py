
import numpy as np
from pressio4py import ode

class MySys:
  def __init__(self):
    pass

  def createVelocity(self):
    print("create")
    return np.zeros(5)

  def velocity(self, stateIn, time, R):
    R[:] = 3.0

class MyDtSetter:
  def __init__(self):
    pass
  def __call__(self, step, time):
    print("MyDtSetter")
    return 1.2

def test_forward_euler_constructor():
  print("\n")
  print("\ntest_forward_euler_constructor")
  state = np.ones(5)
  sys = MySys()
  scheme = ode.stepscheme.ForwardEuler
  stepper = ode.create_explicit_stepper(scheme, state, sys)

def test_forward_euler_call():
  print("\n")
  print("test_forward_euler_call")
  state = np.ones(5)
  sys = MySys()
  scheme = ode.stepscheme.ForwardEuler
  stepper = ode.create_explicit_stepper(scheme, state, sys)
  print("before step: ", state)
  assert( np.all(state == 1.) )
  stepper(state, 0., 2., 1)
  print("after step: ", state)
  '''
  state = state + dt * f
  where f is MySys velocity
  so we should have:
  '''
  assert( np.all(state == 7.) )

def test_forward_euler_advance_n_steps():
  print("\n")
  print("test_forward_euler_advance_n_steps")
  state = np.ones(5)
  sys = MySys()
  scheme = ode.stepscheme.ForwardEuler
  stepper = ode.create_explicit_stepper(scheme, state, sys)
  dt = 1.2
  dtcb = MyDtSetter()
  ode.advance_n_steps(stepper, state, 0., dtcb, 1)
  print("after step: ", state)
  assert( np.all(state == 4.6) )

def test_forward_euler_advance_n_steps_and_observe():
  print("\n")
  print("test_forward_euler_advance_n_steps_and_observe")
  state = np.ones(5)
  sys = MySys()
  scheme = ode.stepscheme.ForwardEuler
  stepper = ode.create_explicit_stepper(scheme, state, sys)
  dt = 1.2

  class MyObs:
    def __init__(self):
      pass
    def __call__(self, step, time, state):
      print("I got cal")
      print(state)
      assert(step == 0 or step == 1)
      if (step ==0):
        assert( np.all(state == 1.) )
      if (step ==1):
        assert( np.all(state == 4.6) )

  ode.advance_n_steps_and_observe(stepper,state,0.,dt,1, MyObs())
  print("after obs: ", state)


#if __name__ == '__main__':
#test_forward_euler()
