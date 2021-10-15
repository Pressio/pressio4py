
# rom: Unsteady LSPG: General Info

\todo: write more

The pressio4py unsteady LSPG ROMs are designed such that
using them involves these main steps:

## 1. Create

You create an instance of a "LSPG problem", e.g.: <br/>

```py
problem = pressio4py.rom.lspg.unsteady.DefaultProblem(args)
```

We currently support three variants:

- Default: [link](md_pages_components_rom_lspg_default.html)
- Hyper-reduced: [link](md_pages_components_rom_lspg_hypred.html)
- Masked: [link](md_pages_components_rom_lspg_masked.html)


All variants return a problem object that meets the following interface:

```py
class UnsteadyLSPGProblem

  def __call__(state, time, time_step_size, step_count, solver);

  def createResidual()
	return # a residual instance

  def createJacobian()
	return # a Jacobian instance

  def residual(state, R)
	# evaluates the residual for the given state

  def jacobian(state, J)
	# evaluates the Jacobian for the given state

  def fomStateReconstructor()
};
```


## 2. Solve in time

What does a stepper have to do with a LSPG ROM?
The answer is that practically speaking, at the lowest-level,
an unsteady LSPG problem can be reduced to simply a "custom" stepper to advance in time.
This is how pressio4py implements this and the reason why a LSPG
problem behaves like a stepper.
You don't need to know how this is done,
or rely on the details, because these are problem- and implementation-dependent,
and we reserve the right to change this in the future.


```py
stepper = ...
pressio4py.ode.advance_n_steps_and_observe(problem, ...)
```

@m_class{m-note m-warning}

@parblock
Remember that for LSPG, you are solving at each step a nonlinear least-squares problem.
Therefore, the solver you need to use is a nonlinear least-squares solver, e.g, Gauss-Newton or Levernberg-Marquardt.
@endparblock
