
# rom: LSPG: steady masked problem

@m_class{m-note m-default}

@parblock
Defined in module: `pressio4py.rom.lspg.steady`

Import as: &emsp; &emsp; &emsp; `from pressio4py.rom import lspg`
@endparblock


## API

```py
problem = lspg.steady.MaskedProblem(fom_adapter, decoder, \
						            rom_state, fom_ref_state, masker)

problem = lspg.steady.PrecMaskedProblem(fom_adapter, decoder, rom_state,
								        fom_ref_state, masker, preconditioner)
```

### Parameters and Requirements

- `fom_adapter`:
  - instance of your adapter class specifying the FOM problem. <br/>
  - must satisfy the [steady API](./md_pages_components_rom_fom_apis.html)

- `decoder`:
  - decoder object
  - must satify the requirements listed [here](md_pages_components_rom_decoder.html)

- `rom_state`:
  - currently, must be a rank-1 `numpy.array`

- `fom_ref_state`:
  - your FOM reference state that is used when reconstructing the FOM state
  - must be a rank-1 `numpy.array`

- `masker`:
  - functor responsible of "masking" the FOM operators
  - must be a functor with a specific API:
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

	def __call__(self, operand, result):
	  result[:] = np.take(operand, self.sample_indices)
  ```
  &nbsp;

- `preconditioner`:
  - functor needed to precondition the ROM operators
  - must be a functor with a specific API:
  ```py
  class Prec:
	def __call__(self, fom_state, operand):
	  # given the current FOM state,
	  # apply your preconditioner to the operand.
	  # Ensure that you overwrite the data in the operand.
	  # As an example, a trivial preconditioner that does nothing:
	  # operand[:] *= 1.
  ```


<br/>
___
<br/>


## Example code

\todo add
