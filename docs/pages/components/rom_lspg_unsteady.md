
# rom: Unsteady LSPG: General Info

\todo: write more

The pressio4py unsteady LSPG ROMs are designed such that
using them involves three main steps:

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

  def stepper()

  def fomStateReconstructor()
};
```

The stepper method returns a reference to an [implicit stepper](md_pages_components_ode_steppers_implicit.html).
The `stepper` method is, practically, what you would use
to retrieve the underlying stepper and use it to solve the problem.
Once you have the stepper, you can then use it as discussed
in [implicit stepper page](md_pages_components_ode_steppers_implicit.html).

What does a stepper have to do with a LSPG ROM?
The answer is that practically speaking, at the lowest-level,
an unsteady LSPG problem can be reduced to simply a "custom" stepper to advance in time.
This is how pressio4py implements this and the reason why a LSPG
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
