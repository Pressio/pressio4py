
# rom: Galerkin: masked problem


@m_class{m-note m-default}

@parblock
Defined in module: `pressio4py.rom.galerkin`

Import as: &emsp; &emsp; &emsp; `from pressio4py.rom import galerkin`
@endparblock


## API, Parameters and Requirements

```py
problem = galerkin.MaskedExplicitProblem(scheme, fom_adapter, decoder,
							             rom_state, fom_ref_state, \      (1)
										 projector, masker)

problem = galerkin.MaskedImplicitProblem(scheme, fom_adapter, decoder,
							             rom_state, fom_ref_state, \      (2)
								         projector, masker)
```

- `scheme`:
  - value from the `ode.stepscheme` enum setting the desired stepping scheme
  - (1) requires [explicit value](md_pages_components_ode_steppers_explicit.html)
  - (2) requires [implicit value](md_pages_components_ode_steppers_implicit.html)

- `fom_adapter`:
  - instance of your adapter class specifying the FOM problem. <br/>
  - must satisfy one of the APIs suitable for Galerkin, see [API list](./md_pages_components_rom_fom_apis.html)

- `decoder`:
  - decoder object
  - must satify the requirements listed [here](md_pages_components_rom_decoder.html)

- `rom_state`:
  - currently, must be a rank-1 `numpy.array`

- `fom_ref_state`:
  - your FOM reference state that is used when reconstructing the FOM state
  - must be a rank-1 `numpy.array`

- `projector`:
  - performs the projection of the FOM operators onto the reduced space
  - must meet [this specific API](md_pages_components_rom_galerkin_projector.html)

- `masker`:
  - an functor responsible of "masking" the FOM operators
  - must be a functor with a specific API, see details below


### Masker

\todo: explain what it is

The masker must meet the following API:

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

where `sample_indices` is a `numpy.array` holding the set of
the row indices to sample.


## Example usage

\todo link tutorials
