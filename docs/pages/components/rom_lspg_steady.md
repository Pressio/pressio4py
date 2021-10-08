
# rom: Steady LSPG

\todo: write this better


The pressio4py steady LSPG ROMs are designed to involve two main steps:

## 1. Create

You instantiate a "steady LSPG problem", e.g.:<br/>

```py
problem = pressio4py.rom.lspg.steady.Problem(...)
```

We currently support two variants:
 - Basic Problem: [link](md_pages_components_rom_lspg_default_steady.html)
 - Masked: [link](md_pages_components_rom_lspg_masked_steady.html)

Refer to each problem page for details on each specific variant.

The returned `problem` object is an instantiation of a class exposing the following interface:

```py
class Problem

  def fomStateReconstructor()
    return # reference to object for reconstructing FOM state

  def createResidual()
	return # a residual instance

  def createJacobian()
	return # a Jacobian instance

  def residual(state, R)
	# evaluates the residual for the given state

  def jacobian(state, J)
	# evaluates the Jacobian for the given state
};
```

## 2. Solve

- you use a nonlinear least-squares solvers to solve the problem
```py
solver = pressio4py.solvers.create_gauss_newton(problem, ...)
solver.solve(problem, ...)
```

- note, in fact, that the problem's API conforms to the one required by the nonlinear solvers

- for this solve stage, you don't have to use the pressio4py solvers.
  Once you have the problem object, you can also use your own nonlinear least-squares solver.
  As shown above, the `problem` exposes all the operators that you need to solve.




<br/>
___
<br/>
