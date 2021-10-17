
# rom: Galerkin: default problem


@m_class{m-note m-default}

@parblock
Defined in module: `pressio4py.rom.galerkin`

Import as: &emsp; &emsp; &emsp; `from pressio4py.rom import galerkin`
@endparblock


## API, Parameters and Requirements

```py
problem = galerkin.DefaultExplicitProblem(scheme, fom_adapter, decoder, \   (1)
									      rom_state, fom_ref_state)

problem = galerkin.DefaultImplicitProblem(scheme, fom_adapter, decoder, \   (2)
									      rom_state, fom_ref_state)
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


<br/>

@m_div{m-button m-primary}
<a href="https://github.com/Pressio/pressio4py/blob/main/demos/unsteady_default_galerkin_advdiff1d_pod/main.py">
@m_div{m-big}Explicit demo @m_enddiv
@m_div{m-small} (click me) @m_enddiv
</a> @m_enddiv
