pressio4py: Projection-based Model Reduction for Python
=======================================================

This package provides Python bindings of the
**projection-based model reduction** C++ library Pressio (website_).
The goal of this project is to enable leading-edge projection-based model
reduction for (large-scale) linear and nonlinear dynamical systems.

.. _website: https://pressio.github.io/pressio/html/index.html


Install
-------
You need to have a C++14-compliant compiler and perform the following two steps:

* create an environment variable to point to your C++ compiler as follows:

  $ export CXX=<path-to-your-compiler>

* install this package using ``pip``::

  $ pip install pressio4py


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
