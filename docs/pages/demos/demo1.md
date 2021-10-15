
# 1D adv-diff: Galerkin with POD modes


@m_class{m-block m-info}

@par
This page describes a demo for a reproductive Galerkin ROM applied to a
1D advection-diffusion problem using POD modes as basis.
By the end, it should be clear how to setup the problem.
This demo purposefully focuses on a simple test since the main goal is
to demonstrate the steps and code. More complex cases will be shown in other demos.
The full demo script is [here.](https://github.com/Pressio/pressio4py/blob/master/demos/unsteady_default_galerkin_advdiff1d_pod/main.py)

## Overview
We cover these three typical steps needed for a ROM:
1. generate of snapshots using the full-order model (FOM)
2. compute the basis: here we demonstrate the use of POD modes
3. execute the ROM: here we leverage the GALERKIN ROM to demonstrate
a *reproductive* test, i.e., we run the ROM using the same physical coefficients, b.c., etc.
A predictive run is demonstrated in a different demo.

## FOM Equations
The governing equations for this problem are:

@f[
\frac{\partial u}{\partial t}
= \frac{\partial}{\partial x} (k(u,x) \frac{\partial u}{\partial x} )
- a*\frac{\partial u}{\partial x}
@f]
where @f$k(u,x)=x^4@f$, the field is @f$u(x;t)@f$, the advection velocity
is fixed at @f$a=2@f$, the spatial coordinate is @f$x@f$ and the domain is @f$(0,1)@f$.
We use homogeneous BC. Note that a class approximating the FOM operators via finite-differences
is implemented [here](https://github.com/Pressio/pressio4py/blob/master/apps/adv_diff1d.py).


## Main function
The main function of the demo is the following:
```py
@codesnippet
../../../demos/unsteady_default_galerkin_advdiff1d_pod/main.py
59:93
```

### 1. Run FOM and collect snapshots
```py
@codesnippet
../../../demos/adv_diff_1d_fom.py
5:17
```

### 2. Compute POD modes
```py
@codesnippet
../../../demos/unsteady_default_galerkin_advdiff1d_pod/main.py
18:21
```

### 3. Construct and run ROM
```py
@codesnippet
../../../demos/unsteady_default_galerkin_advdiff1d_pod/main.py
23:56
```

## Results
If everything works fine, the following plot shows the result.
We see that for this toy example, the full solution is recovered very well with Galerkin
with just a few POD modes.
@image html demo1.png
