
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


All variants return a problem object that meets the following interface:

```py
class GalerkinProblem

  def stepper()

  def fomStateReconstructor()
};
```

The stepper method returns a reference to an
[explicit stepper](md_pages_components_ode_steppers_explicit.html) or
[implicit stepper](md_pages_components_ode_steppers_implicit.html),
depending on what you pass when you create the Galerkin problem.
The `stepper` method is, practically, what you would use
to retrieve the underlying stepper and use it to solve the problem.
Once you have the stepper, you can then use it as discussed
on the [explicit stepper page](md_pages_components_ode_steppers_explicit.html)
or [implicit stepper page](md_pages_components_ode_steppers_implicit.html).

What does a stepper have to do with a Galerkin ROM?
The answer is that practically speaking, at the lowest-level,
a Galerkin problem can be reduced to simply a "custom" stepper to advance in time.
This is how pressio4py implements this and the reason why a Galerkin
problem contains a stepper object inside: when you create the
problem, pressio creates the appropriate custom stepper
object that you can use. You don't need to know how this is done,
or rely on the details, because these are problem- and implementation-dependent,
and we reserve the right to change this in the future.


## 2. Reference the stepper and solve in time

Extract the underlying stepper object and solve in time:

```py
stepper = problme.stepper()
pressio4py.ode.advance_n_steps_and_observe(stepper, ...)
```
