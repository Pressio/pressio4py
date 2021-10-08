
# ode advancers


@m_class{m-note m-default}

@parblock
Defined in module: `pressio4py.ode`

Import as: &emsp; &emsp; &emsp; `from pressio4py import ode`
@endparblock


## API

@m_class{m-note m-info}

@parblock
Overload set for advancing for fixed number of steps
@endparblock


```py
advance_n_steps(stepper, state, start_time, \
				time_step_size, num_steps);						    (1)

advance_n_steps(stepper, state, start_time, \
				time_step_setter, num_steps);					    (2)

advance_n_steps_and_observe(stepper, state, start_time, \
						    time_step_size, num_steps, observer);   (3)

advance_n_steps_and_observe(stepper, state, start_time, \
							time_step_setter, num_steps, observer); (4)
```

- (1,2): overloads for advancing for a fixed number of steps
- (3,4): overloads for advancing for a fixed number of steps accepting
also an "observer" to monitor the evolution of the state at each step (more on this below)


## Parameters and Requirements

- `stepper`: the steppable object, see e.g.:
  - [explicit steppers](md_pages_components_ode_steppers_explicit.html)
  - stepper extracted from a [Galerkin ROM problem](md_pages_components_rom_galerkin_default.html)
  - stepper extracted from a [LSPG ROM problem](md_pages_components_rom_lspg_default.html)

- `state`: must be a `numpy.array`

- `start_time`: self-explanatory

- `num_steps`: self-explanatory

- `time_step_size`:
  - size of the time step to use at each step (fixed)
  - applicable only to overloads (1,2)

- `time_step_setter`:
  - applicable only to overloads (3,4)
  - a functor responsible for setting the time step size to use at a given step
  ```py
  class MyStepSizeSetter:
	def __call__(self, step, time):
      # set time step and return
	  dt = 1.5
      return dt
  ```

- `observer`:
  - functor that you use to "observe" the state during the
  time integration. This is useful for collecting snapshots of the state, or
  necessary data/metrics/statistics.<br/>
  - Must expose at least the call operator as follows:
  ```py
  class MyObs:
	def __call__(self, step, time, state):
	  # do something with state
  ```


@m_class{m-note m-info}

@parblock
By design, the steps are enumerated as follows: `1,2,3...,num_steps`.
Therefore, step 1 is the step that starts at `t_0 = start_time` and ends at `t_1 = start_time + dt_1`,
step 2 is the step that starts at `t_1` and ends at `t_2 = t_1 + dt_2`.
Here, `dt_n` indicates the time step size to use for the `n`-th step.
@endparblock
