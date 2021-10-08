
# rom: Steady LSPG

\todo: write this better


The pressio4py steady LSPG ROMs are designed such that
using them boils down to two main steps:

## 1. Create

You instantiate a "steady LSPG problem", e.g.:<br/>

```py
problem = pressio4py.rom.lspg.steady.Problem(...)
```

We currently support two variants:
 - Basic Problem: [link](md_pages_components_rom_lspg_default_steady.html)
 - Masked: [link](md_pages_components_rom_lspg_masked_steady.html)


The `problem` class exposes the following interface:

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
- note how the API conforms to the one required by the nonlinear solvers


## 2. Solve

- you use a nonlinear least-squares solvers to solve the problem
```py
solver = pressio4py.solvers.create_gauss_newton(problem, ...)
solver.solve(problem, ...)
```
- note that you don't have to use our solvers, you can also use your own.
The `problem` exposes everything you need to solve.


Refer to each problem page for details on each specific variant.

<br/>
___
<br/>
