
# 1D adv-diff: LSPG with nonlinear manifold projection via kPCA


@m_class{m-block m-info}

@par What does this page describe?
This page describes a demo for a reproductive LSPG ROM applied to a
1D advection-diffusion problem using a nonlinear manifold via kernel PCA.
This demo purposefully focuses on a simple test since the main goal is
to demonstrate the steps and the code.
The full demo script is [here.](https://github.com/Pressio/pressio4py/blob/master/demos/unsteady_default_lspg_advdiff1d_kpca/main.py)


## Overview
This demo solves the same problem as the one
[here](https://pressio.github.io/pressio4py/html/md_pages_demos_demo1.html),
but instead of using POD modes, we show here how to use
a nonlinear manifold computed via kernel PCA.

<!-- ## Imports -->
<!-- The imports needed are: -->
<!-- ```py -->
<!-- from adv_diff1d import *					# the fom class -->
<!-- from adv_diff_1d_fom import doFom			# the function to collect fom data -->
<!-- from pressio4py import rom as rom -->
<!-- from pressio4py import solvers as solvers -->
<!-- ``` -->

## Main function
The main function of the demo is the following:
```py
@codesnippet
../../../demos/unsteady_default_lspg_advdiff1d_kpca/main.py
105:130
```

### 1. Run FOM and collect snapshots
This step is the same as described [here](https://pressio.github.io/pressio4py/html/md_pages_demos_demo1.html),


### 2. Setup and train the nonlinear kPCA mapper
It is important to note that while the mapper class below has
the API required by pressio4py, it can encapsulate any arbitrary mapping function.
In this case we show how to create a kPCA-based representation, but one
can use, e.g., autoencoder, and any other types of mapping.
This is how we enable support for testing various methods.
```py
@codesnippet
../../../demos/unsteady_default_lspg_advdiff1d_kpca/main.py
20:56
```

@m_class{m-block m-warning}

@par Important:
when creating an arbitrary mapping (as in the class above),
the jacobian matrix **must** be column-major oder so that pressio
can reference it without deep copying it. This not only reduced the
memory footprint since it allows to keep only one jacobian object
around but also it is fundamental for the update method below correctly.


### 3. Construct and run LSPG
```py
@codesnippet
../../../demos/unsteady_default_lspg_advdiff1d_kpca/main.py
59:102
```

## Results
If everything works fine, the following plot shows the result.
@image html tutorial3.png
