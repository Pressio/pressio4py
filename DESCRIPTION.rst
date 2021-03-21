pressio4py: Projection-based Model Reduction for Scientific Computing
=====================================================================

This package provides Python bindings for the
**projection-based model reduction** C++ library Pressio (website_).
The goal of this project is to enable leading-edge projection-based model
reduction for (large-scale) linear and nonlinear dynamical systems.

.. _website: https://pressio.github.io/pressio/html/index.html


Install
-------

You need a C++14-compliant compiler and then do:

.. code-block:: bash

  export CXX=<path-to-your-C++-compiler>
  pip install pressio4py


You can double check that everything worked fine by doing:

.. code-block:: python

  import pressio4py
  print(pressio4py.__version__)



Running Demos/Tutorials
-----------------------

After installing the library, you can run the regression tests:

.. code-block:: bash

  git clone --recursive git@github.com:Pressio/pressio4py.git
  cd pressio4py
  pytest -s


And you can check out the demos:

.. code-block:: bash

  git clone --recursive git@github.com:Pressio/pressio4py.git
  cd pressio4py/demos
  python3 ./<demo-subdir-name>/main.py


Documentation
-------------

The documentation (in progress) can be found (here_) with some demos already available.

.. _here: https://pressio.github.io/pressio4py/html/index.html


Citations
---------

If you use this package, please acknowledge our work-in-progress:

* Francesco Rizzi, Patrick J. Blonigan, Eric. Parish, Kevin T. Carlberg
  "Pressio: Enabling projection-based model reduction for large-scale nonlinear dynamical systems"
  https://arxiv.org/abs/2003.07798
