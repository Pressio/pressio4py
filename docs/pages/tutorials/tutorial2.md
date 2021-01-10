
# Tutorial: Custom Decoder

@m_class{m-block m-info}

@par
This tutorial shows how to create a *custom* decoder in pressio4py.

## Context
A custom decoder in pressio4py implements a general mapping:
@f[
y_{fom} = g(y_{rom})
@f]
where @f$y_{rom}@f$ is the reduced state, also called
generalized coordinates, @f$y_{fom}@f$ is the
full-order model (FOM) state,
and @f$g@f$ is the mapping between the two.
The Jacobian of the mapping is:
@f[
J_g = \frac{d g}{d y_{rom}}.
@f]

This allows one to use an arbitrary function to map the ROM state to
the FOM state.

## Code
The full tutorial can be found [here](https://github.com/Pressio/pressio4py/blob/master/tutorials/tut_custom_decoder/main.py)

```py
@codesnippet
../../../tutorials/tut_custom_decoder/main.py
2:39
```


@m_class{m-block m-warning}

@par Where can you use a custom decoder?
Currently, a custom decoder can only be used for LSPG.
See [this demo](https://pressio.github.io/pressio4py/html/md_pages_demos_demo3.html) for an example usage.
