

# Tutorial: Linear Decoder

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
where @f$\phi@f$ is a matrix (for the time being assumed constant).
The Jacobian of the mapping is:
@f[
\frac{d y_{fom}}{d y_{rom}} = \phi.
@f]

A linear decoder in pressio4py represents this linear mapping.

Note that the above expressions are abtract, since they do not specify
what kind of data structure is used for @f$y_{fom}@f$.
We envision the following scenarios:
* rank-1 state: the FOM state is stored as an array, @f$y_{fom} \in R^N@f$,
where @f$N@f$ = **total number of degrees of freedom**.
In this case, the application possibly involves multiple fields (e.g., density, chemical species, etc),
but stores in a single array all the spatial degrees of freedom.
this is common when the application needs to do implicit time-integration
such that a large system needed to be solved.

* the FOM state is represented as rank-2 tensor, i.e. @f$y_{fom} \in R^N@f$


## Rank-1 state
The full tutorial can be found [here](https://github.com/Pressio/pressio4py/blob/master/tutorials/tut_linear_decoder/main.py)

```py
@codesnippet
../../../tutorials/tut_linear_decoder/main.py
2:23
```


@m_class{m-block m-warning}

@par Where can you use the linear decoder?
A linear decoder can be used for both Galerkin and LSPG
as shown in subsequent tutorials and in the demos.
