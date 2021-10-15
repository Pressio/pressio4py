
# 1D adv-diff: Comparing Masked POD Galerkin against masked POD LSPG


@m_class{m-block m-info}

@par
This page describes a demo for a reproductive "masked" Galerkin and LSPG ROMs
applied to a 1D advection-diffusion problem using POD modes as basis.
The full demo script is [here.](https://github.com/Pressio/pressio4py/blob/master/demos/unsteady_masked_galerkin_vs_lspg_advdiff1d_pod/main.py)

@m_class{m-block m-warning}

@par We are currently working on this page, it will be updated with more explanations.


## Overview
This is a follow up to the previous demo [here](./md_pages_demos_demo4.html)
We compare here maskdd Galerkin and masked LSPG.


## Main function
The main function of the demo is the following:
```py
@codesnippet
../../../demos/unsteady_masked_galerkin_vs_lspg_advdiff1d_pod/main.py
161:225
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
../../../demos/unsteady_masked_galerkin_vs_lspg_advdiff1d_pod/main.py
43:46
```

### 3. Create the sampling indices
```py
@codesnippet
../../../demos/unsteady_masked_galerkin_vs_lspg_advdiff1d_pod/main.py
184:201
```

### 4. The masker class
```py
@codesnippet
../../../demos/unsteady_masked_galerkin_vs_lspg_advdiff1d_pod/main.py
25:40
```

### 5. Masked Galerkin ROM
```py
@codesnippet
../../../demos/unsteady_masked_galerkin_vs_lspg_advdiff1d_pod/main.py
57:111
```

### 6. Masked LSPG ROM
```py
@codesnippet
../../../demos/unsteady_masked_galerkin_vs_lspg_advdiff1d_pod/main.py
113:158
```

## Results
If everything works fine, the following plots shows the result.
We first plot the result reconstructed *only on the sample mesh*.
This can easily be done using the bases collocated on the sample mesh indices.
@image html demo5_f1.png

We then plot the fom solution reconstructed using the bases on the full mesh.
Note that all we need to change is just using the full bases.
We see that for this toy example, even with just 10% of the grid, LSPG
with 5 modes accuractely reproduces the FOM solution.
While for Galerkin the solution is less accurate.
@image html demo5_f2.png


@m_class{m-block m-warning}

@par Warning
Note that using the mask to mimic hyper-reduction is
only helpful to assess the accuracy but not the computational performance.
This is because the "masked" problem still requires the FOM
to compute the full kernels. Hyper-reduction becomes computationally
very efficient if implemented without the mask,
which we will show in subsequent demos.
