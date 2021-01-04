
# Tutorial: Linear Decoder

@m_class{m-block m-info}

@par Content
This tutorial shows how to create a linear decoder in pressio4py.

## Context
A key assumption of projection-based ROMs relies on approximating
a full-order model (FOM) state, @f$y_{fom}@f$, as:
@f[
y_{fom} = g(y_{rom})
@f]

where @f$y_{rom}@f$ is the reduced state, also called
generalized coordinates, and @f$g@f$ is the mapping between the two.
If @f$g@f$ is linear, then we can write:
@f[
y_{fom} = \phi y_{rom}
@f]
where @f$\phi@f$ is a matrix (for the time being, assume it constant).
A linear decoder in pressio4py implements this linear mapping.
Due to the linearity, the Jacobian of the mapping is:
@f[
\frac{d y_{fom}}{d y_{rom}} = \phi.
@f]

## Code
Here we demonstate how to create a linear decoder.
The full tutorial can be found [here](https://github.com/Pressio/pressio4py/blob/master/tutorials/tut_linear_decoder/main.py)

```py
@codesnippet
../../../tutorials/tut_linear_decoder/main.py
12:45
```


@m_class{m-block m-info}

@par Where can you use the linear decoder?
A linear decoder can be used for both Galerkin and LSPG
as will be shown in subsequent tutorials and in the demos.
