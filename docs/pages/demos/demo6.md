
# 1D adv-diff: LSPG with nonlinear manifold projection via MLP


@m_class{m-block m-info}

@par
This page describes a demo for a reproductive LSPG ROM applied to a
1D advection-diffusion problem using a nonlinear manifold via a multilayer perceptron (MLP).
This demo purposefully focuses on a simple test since the main goal is
to demonstrate the steps and the code.
The full demo script is [here](https://github.com/Pressio/pressio4py/blob/master/demos/unsteady_default_lspg_advdiff1d_mlp/main.py).

@m_class{m-block m-warning}

@par Important:
The MLP used in this demo is implemented in PyTorch and thus PyTorch must be installed prior to executing this demo.


## Overview
This demo solves the same problem as the one
[here](https://pressio.github.io/pressio4py/html/md_pages_demos_demo1.html),
but instead of using POD modes, we show here how to use
a nonlinear manifold computed approximated by a neural network.
Specifically, we use a MLP with 2 hidden layers of sizes 64 and 200.

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
../../../demos/unsteady_default_lspg_advdiff1d_mlp/main.py
66:94
```

### 1. Run FOM and collect snapshots
This step is the same as described [here](https://pressio.github.io/pressio4py/html/md_pages_demos_demo1.html),


### 2. Setup and train the nonlinear mapper
It is important to note that while the mapper class below has
the API required by pressio4py, it can encapsulate any arbitrary mapping function.
In this case we show how to create a MLP-based representation in PyTorch, but one
can use any other types of mapping and any other library (e.g., Tensorflow, keras).
All of the PyTorch-specific code is encapsulated [here](https://github.com/Pressio/pressio4py/blob/master/demos/unsteady_default_lspg_advdiff1d_mlp/autoencoder_PyTorch.py).

The autoencoder is defined by
```py
@codesnippet
../../../demos/unsteady_default_lspg_advdiff1d_mlp/autoencoder_PyTorch.py
8:63
```

and is created/trained using
```py
@codesnippet
../../../demos/unsteady_default_lspg_advdiff1d_mlp/autoencoder_PyTorch.py
111:134
```

This is all wrapped in a mapper class which conforms to the API required by Pressio
```py
@codesnippet
../../../demos/unsteady_default_lspg_advdiff1d_mlp/autoencoder_PyTorch.py
65:108
```

@m_class{m-block m-warning}

@par Important:
when creating an arbitrary mapping (as in the class above),
the jacobian matrix **must** be column-major oder so that pressio
can reference it without deep copying it. This not only reduces the
memory footprint since it allows to keep only one jacobian object
around but also it is fundamental for the update method below correctly.


### 3. Construct and run LSPG
```py
@codesnippet
../../../demos/unsteady_default_lspg_advdiff1d_mlp/main.py
20:63
```

## Results
If everything works fine, the following plot shows the result.
@image html demo6.png
