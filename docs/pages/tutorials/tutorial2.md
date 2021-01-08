
# Tutorial: Custom Decoder

@m_class{m-block m-info}

@par Content
This tutorial shows how to create a *custom* decoder in pressio4py.

## Context
A key assumption of projection-based ROMs relies on approximating
a full-order model (FOM) state, @f$y_{fom}@f$, as:
@f[
y_{fom} = g(y_{rom})
@f]
where @f$y_{rom}@f$ is the reduced state, also called
generalized coordinates, and @f$g@f$ is the mapping between the two.
The Jacobian of the mapping is:
@f[
J_g = \frac{d g_{fom}}{d y_{rom}}.
@f]
A custom decoder in pressio4py implements this general mapping.
This allows one to use an arbitrary function to map the ROM state to
the FOM state.

## Code
Here we demonstate how to create a linear decoder.
The full tutorial can be found [here](https://github.com/Pressio/pressio4py/blob/master/tutorials/tut_custom_decoder/main.py)

```py
@codesnippet
../../../tutorials/tut_custom_decoder/main.py
2:39
```


@m_class{m-block m-warning}

@par Where can you use a custom decoder?
Currently, a custom decoder can only be used for LSPG.
