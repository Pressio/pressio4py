
# rom: Galerkin: General Info

\todo: write more

The pressio4py Galerkin ROMs are designed such that
using them involves three main steps:

## 1. Create

You create an instance of a "Galerkin problem", e.g.: <br/>

```py
problem = pressio4py.rom.galerkin.DefaultExplicitProblem(args)
```

We currently support three variants:

- Default: [link](md_pages_components_rom_galerkin_default.html)
- Hyper-reduced: [link](md_pages_components_rom_galerkin_hypred.html)
- Masked: [link](md_pages_components_rom_galerkin_masked.html)


The `problem` object behaves like a stepper.
Therefore, you can use the problem like
you would with any other stepper object (more on this below).

### Explicit Problem

The problem meets the following API:

```py
class GalerkinProblem

  def __call__(state, time, time_step_size, step_count);

  def fomStateReconstructor();
};
```

### Implicit Problem

The problem meets the following API:

```py
class GalerkinProblem

  def __call__(state, time, time_step_size, step_count, solver);

  def createResidual()
	return # a residual instance

  def createJacobian()
	return # a Jacobian instance

  def residual(state, R)
	# evaluates the residual for the given state

  def jacobian(state, J)
	# evaluates the Jacobian for the given state

  def fomStateReconstructor();
};
```

## 2. Solve in time

What does a stepper have to do with a Galerkin ROM problme?
The answer is that practically speaking, at the lowest-level,
a Galerkin problem can be reduced to simply a "custom" stepper to advance in time.
This is how pressio4py implements this and the reason why a Galerkin
problem contains a stepper object inside: when you create the
problem, pressio creates the appropriate custom stepper
object that you can use. You don't need to know how this is done,
or rely on the details, because these are problem- and implementation-dependent,
and we reserve the right to change this in the future.

```py
problem = ...
pressio4py.ode.advance_n_steps_and_observe(problem, ...)
```
