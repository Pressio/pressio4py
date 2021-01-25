

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
what kind of data structures are used.
pressio4py supports two scenarios:
1. the FOM and ROM states are *rank-1 tensors* (i.e., vectors)
2. the FOM and ROM states are *rank-2 tensors* (i.e., matrices)


## Rank-1 state
The full tutorial can be found [here](https://github.com/Pressio/pressio4py/blob/master/tutorials/tut_linear_decoder/main.py).

The FOM state is stored as an array, @f$y_{fom} \in R^N@f$,
where @f$N@f$ = **total number of degrees of freedom**.
In this case, the application possibly involves multiple fields (e.g., density, chemical species, etc),
but stores all the spatial degrees of freedom in a single array.
For example, in a finite-volume code, one stores contiguously all field values of a given cell, for all cells.
This is common, for example, when the application needs to do implicit time-integration
such that a large system needed to be solved.
@image html tut_lindec_f1.png width=50px

```py
@codesnippet
../../../tutorials/tut_linear_decoder/main.py
2:23
```

@m_class{m-block m-warning}

@par Where can you use the linear decoder with a rank-1 state?
It can be employed for both Galerkin and LSPG
as shown in subsequent tutorials and in the demos.



## Rank-2 state
The full tutorial can be found [here](https://github.com/Pressio/pressio4py/blob/master/tutorials/tut_linear_decoder/main.py).

In this case, the FOM state is stored as a matrix, @f$y_{fom} \in R^{N,m}@f$, where:
* @f$m@f$ = the total number of fields. E.g., density, x-velocity, tracer concentration, etc.
* @f$N@f$ = number of degrees of freedom of each field

This scenario is intended for applications that prefer to separate the degress of freedom,
and is more commonly found for explicit time integration.


@m_class{m-block m-warning}

@par Where can you use the linear decoder with a rank-1 state?
It can be employed for both Galerkin and LSPG
as shown in subsequent tutorials and in the demos.
