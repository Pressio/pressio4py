
# Tutorial: Linear Decoder

@m_class{m-block m-info}

@par
This tutorial shows how to create a linear decoder in pressio4py.

## Context
A key assumption of projection-based ROMs relies on approximating
a full-order model (FOM) state, @f$fomState@f$, as:
@f[
fomState = g(romState)
@f]

where @f$romState@f$ is the reduced state, also called
generalized coordinates, and @f$g@f$ is the mapping between the two.
If @f$g@f$ is linear, then we can write:
@f[
fomState = \phi romState
@f]
where @f$\phi@f$ is a matrix (for the time being, assume it constant).
Due to the linearity, the Jacobian of the mapping is:
@f[
\frac{d fomState}{d romState} = \phi.
@f]

A linear decoder in pressio4py represents this linear mapping.

## Code
The full tutorial can be found [here](https://github.com/Pressio/pressio4py/blob/master/tutorials/tut_linear_decoder/main.py)

```py
@codesnippet
../../../tutorials/tut_linear_decoder/main.py
2:37
```


@m_class{m-block m-warning}

@par Where can you use the linear decoder?
A linear decoder can be used for both Galerkin and LSPG
as will be shown in subsequent tutorials and in the demos.
