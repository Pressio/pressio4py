
# rom: LSPG: unsteady masked problem


@m_class{m-note m-default}

@parblock
Defined in module: `pressio4py.rom.lspg.unsteady`

Import as: &emsp; &emsp; &emsp; `from pressio4py.rom import lspg`
@endparblock


## API, Parameters and Requirements

```py
# continuous-time overloads
problem = lspg.unsteady.MaskedProblem(scheme, fom_adapter, decoder, \              (1)
								      rom_state, fom_ref_state, masker)

problem = lspg.unsteady.PrecMaskedProblem(scheme, fom_adapter, decoder, \
									      rom_state, fom_ref_state,     \          (2)
										  masker, preconditioner)

# discrete-time overloads
problem = lspg.unsteady.DiscreteTimeProblemTwoStates(fom_adapter, decoder,    \	   (3)
													 rom_state, fom_ref_state,\
													 masker)

problem = lspg.unsteady.DiscreteTimeProblemThreeStates(fom_adapter, decoder,    \
													   rom_state, fom_ref_state,\  (4)
													   masker)
```

- `scheme`:
  - only applicable to (1,2)
  - value from the `ode.stepscheme` enum setting the desired stepping scheme
  - requires an [implicit value](md_pages_components_ode_steppers_implicit.html)

- `fom_adapter`:
  - for (1,2): must statisfy the continuous-time API for unsteady LSPG, see [API list](./md_pages_components_rom_fom_apis.html)
  - for (3): must satisfy the discrete-time API with two states, see [API list](./md_pages_components_rom_fom_apis.html)
  - for (4): must satisfy the discrete-time API with three states, see [API list](./md_pages_components_rom_fom_apis.html)

- `decoder`:
  - decoder object
  - must satify the requirements listed [here](md_pages_components_rom_decoder.html)

- `rom_state`:
  - currently, must be a rank-1 `numpy.array`

- `fom_ref_state`:
  - your FOM reference state that is used when reconstructing the FOM state
  - must be a rank-1 `numpy.array`

- `masker`:
  - an functor responsible of "masking" the FOM operators
  - must be a functor with a specific API, see details below
  ```py
  class Masker:
	def __init__(self, sample_indices):
	  self.sample_indices = sample_indices
	self.N = len(self.sample_indices)

	def createApplyMaskResult(self, operand):
		if (operand.ndim == 1):
		  return np.zeros(N)
		else:
		  return np.zeros((N, , operand.shape[1]))

	def __call__(self, operand, time, result):
	  # time is not used, but you can potentially
	  result[:] = np.take(operand, self.sample_indices)
  ```
  &nbsp;

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
