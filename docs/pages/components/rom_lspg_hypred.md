
# rom: LSPG: unsteady hyper-reduced problem

@m_class{m-note m-default}

@parblock
Defined in module: `pressio4py.rom.lspg.unsteady`

Import as: &emsp; &emsp; &emsp; `from pressio4py.rom import lspg`
@endparblock


## API, Parameters and Requirements

```py
problem = lspg.unsteady.HypredProblem(scheme, fom_adapter, decoder, \
								      rom_state, fom_ref_state, \
									  sampleToStencilIndexing)

problem = lspg.unsteady.PrecHypredProblem(scheme, fom_adapter, decoder, \
									      rom_state, fom_ref_state, \
										  sampleToStencilIndexing, preconditioner)
```

- `scheme`:
  - value from the `ode.stepscheme` enum setting the desired stepping scheme
  - requires an [implicit value](md_pages_components_ode_steppers_implicit.html)

- `fom_adapter`:
  - instance of your adapter class specifying the FOM problem. <br/>
  - must be admissible to unsteady LSPG, see [API list](./md_pages_components_rom_fom_apis.html)

- `decoder`:
  - decoder object
  - must satify the requirements listed [here](md_pages_components_rom_decoder.html)

- `rom_state`:
  - currently, must be a rank-1 `numpy.array`

- `fom_ref_state`:
  - your FOM reference state that is used when reconstructing the FOM state
  - must be a rank-1 `numpy.array`

- `stencilToSampleIndexing`:
  - an object that knows the mapping from sample to stancil operators
  - can be constructed as: <br/>
  ```py
  mapper = rom.lspg.unsteady.StencilToSampleIndexing(list_of_ints)
  ```
  &nbsp;
  - see section below for more details

- `preconditioner`:
  - functor needed to precondition the ROM operators
  - must be a functor with a specific API:
  ```py
  class Prec:
	def __call__(self, fom_state, time, operand):
	  # given the current FOM state,
	  # apply your preconditioner to the operand.
	  # Ensure that you overwrite the data in the operand.
	  # As an example, a trivial preconditioner that does nothing:
	  # operand[:] *= 1.
  ```
<br/>

## Stencil to sample indexing

When working with a hyper-reduced problem, pressio4py has to manipulate objects
that have different sizes/distributions.
For such problem, in fact, some operators are naturally defined
on the what we refer to as "sample mesh" while some are defined on what
we call the "stencil mesh".

As explained [here](https://pressio.github.io/algos/hyper/), recall that:

1. **sample mesh**: a disjoint collection of elements where the velocity (or residual) operator is computed.

2. **stencil mesh**: the set of all nodes or elements needed to compute the velocity or residual on the sample mesh.

3. Typically, the sample mesh is a subset of the stencil mesh.

<!-- For more details on this and hyper-reduction in general, -->
<!-- see [this page](https://pressio.github.io/algos/hyper/). -->

@m_class{m-note m-info}

@parblock
The sample to stencil indexing is a list of indices that you need to provide such that
pressio4py knows how to properly combine operands defined on stencil and sample mesh.
@endparblock


### Explain it to me better!

Suppose that your FOM problem involves a 2D problem and that your FOM numerical
method needs at every cell information from the nearest neighbors.
For the sake of explanation, *it does not matter what problem we are solving*,
only what we just said.
Now, suppose that you want to try hyper-reduced LSPG on it.
You come up with a sample and stencil mesh for your problem
(read [this page](https://pressio.github.io/algos/hyper/) for some information about
how to select sample mesh cells), and let's say it looks like this:
@image html lspg_sample_mesh1.png width=400px

The stencil mesh is the set of *all* cells shown,
while the sample mesh is the *subset* color-coded yellow.
We have added an arbitrary enumeration scheme to uniquely assign a global index to each cell.
The enumeration order does not matter, this is just for demonstration purposes.
You have an adapter class for your problem that is able to compute
the FOM right-hand-side @f$f@f$ on the yellow cells, for a given
FOM state @f$y@f$ on the stencil mesh.

For this example, you then would do this:

```py
# ...
mylist = [1,4,9,14,18,24,25,31,37,40,47,50,53,62,65,70]
indexing = rom.lspg.unsteady.StencilToSampleIndexing(mylist)
scheme = ode.stepscheme.BDF1
lspgProblem = rom.lspg.unsteady.HypredProblem(..., indexing)
# ...
```

@m_class{m-block m-info}

@par Note that:
- how you enumerate the cells does not matter.
  You are free to do whatever you want, as long as your adapter object is
  consistent with the chosen enumeration scheme and handles things accordingly.

- This indexing notion seamlessly extends to 1D and 3D problems.
@endparblock


<!-- Now, suppose that you want to do LSPG with BDF1. In such case, pressio4py has to compute: -->
<!-- @f[ -->
<!-- R = y_{n+1}-y_{n}- hf(t_{n+1},y_{n+1}) -->
<!-- @f] -->
<!-- where @f$y@f$ is a FOM state, @f$f@f$ the right-hand-size, @f$t@f$ is time, and @f$R@f$ is the residual. -->
