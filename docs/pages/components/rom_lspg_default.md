
# rom: LSPG: unsteady default problem


@m_class{m-note m-default}

@parblock
Defined in module: `pressio4py.rom.lspg.unsteady`

Import as: &emsp; &emsp; &emsp; `from pressio4py.rom import lspg`
@endparblock


## API, Parameters and Requirements

```py
problem = lspg.unsteady.DefaultProblem(scheme, fom_adapter, decoder, \
									   rom_state, fom_ref_state)

problem = lspg.unsteady.PrecDefaultProblem(scheme, fom_adapter, decoder, \
									   rom_state, fom_ref_state, preconditioner)
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
