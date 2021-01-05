
# 1D adv-diff: LSPG with POD modes


@m_class{m-block m-info}

@par What does this page describe?
This page describes a demo for a reproductive LSPG ROM applied to a
1D advection-diffusion problem using POD modes as basis.
By the end, it should be clear how to setup the problem.
This demo purposefully focuses on a simple test since the main goal is
to demonstrate the steps and the code.
The full demo script is [here.](https://github.com/Pressio/pressio4py/blob/master/demos/unsteady_default_lspg_advdiff1d_pod/main.py)


## Overview
We cover these three typical steps needed for a ROM:
1. generate of snapshots using the full-order model (FOM)
2. compute the basis: here we demonstrate the use of POD modes
3. execute the ROM: here we leverage the LSPG ROM to demonstrate
a *reproductive* test, i.e., we run the ROM using the same physical coefficients, b.c., etc.
A predictive run is demonstrated in a different tutorial.

The governing equations for this problem are the same
as those in [here](https://pressio.github.io/pressio4py/html/md_pages_demos_demo2.html),


## Main function
The main function of the demo is the following:
```py
@codesnippet
../../../demos/unsteady_default_lspg_advdiff1d_pod/main.py
74:103
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
../../../demos/unsteady_default_lspg_advdiff1d_pod/main.py
18:21
```

### 3. Construct and run ROM
```py
@codesnippet
../../../demos/unsteady_default_lspg_advdiff1d_pod/main.py
24:71
```

## Results
If everything works fine, the following plot shows the result.
@image html demo2.png
