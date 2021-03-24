
# Python Bindings for Pressio

*Leading-edge projection-based reduced order models (\proms) for
dynamical systems in science and engineering.*

This is the documentation of the Python bindings library,
which is one component of the [Pressio project](https://pressio.github.io/).

## Install

@m_class{m-block m-warning}

@par
```bash
# the C++ compiler must support C++14
export CXX=<path-to-your-C++-compiler>
git clone --recursive git@github.com:Pressio/pressio4py.git # or https://github.com/Pressio/pressio4py.git
pip install ./pressio4py

# you can then run the regression tests:
cd pressio4py
pytest -s
```
Note that if you get an import error, it might be that
the version of `pytest` you are
using is not compatible with the `pip` command you used to install.
Make sure you use Python commands from the **same** distribution.


## In a nutshell

Pressio can be applied to any dynamical system expressible in
a *continuous-time* form as
@f[
\frac{d \boldsymbol{y}}{dt} =
\boldsymbol{f}(\boldsymbol{y},t; ...)
@f]
and/or in a *discrete-time* form
@f[
\boldsymbol{R}(\boldsymbol{y}, \boldsymbol{y_{n-1}}, ..., t_n, dt_n; ...) = \boldsymbol{0}
@f]

Here, @f$y@f$ is the full-order model (FOM) state,
@f$f@f$ the FOM velocity, @f$t@f$ is time, and @f$R@f$ is the residual.

This formulation is quite general and does not make any assumption
on its origin: it may be derived from the spatial
discretization (regardless of the discretization method)
of a PDE problem, or from naturally discrete systems.

We leverage this expressive mathematical framework as a pivotal
design choice to enable a minimal application programming interface (API)
that is natural to dynamical systems: you choose the formulation
more convenient to you, and interface your application to
Pressio by creating a corresponding *adapter class* to expose
the operators needed for the chosen formulation.
In general, you don't need to support both: each one has advantages and disadvantages,
and sometimes the choice is dictated directly by your native application (for example,
in some cases it might be easier to directly expose the discrete-time residual).

Read [this doc page](./md_pages_prepare_your_app.html)
to learn more about the adapter classes and see code templates.


## Explore the tutorials and demos
You can find descriptions of the demos [here](./md_pages_demos_demo1.html)
and of the tutorials [here](./md_pages_tutorials_tutorial1.html)---we will progressively add more.
```bash
cd pressio4py/demos
python3 ./<demo-subdir-name>/main.py
```


<!-- read the [building/installation process](./md_pages_getstarted_build_and_install.html)>
<!-- Untill we start filling the tutorials and examples, you can peek at the [test subdirectory](https://github.com/Pressio/pressio/tree/master/tests/rom/burgers1d) of the C++ library. -->


## License and Citation
The full license is available [here](https://pressio.github.io/various/license/).

We are working on publishing this: you can find our arXiv preprint at: https://arxiv.org/abs/2003.07798

## Questions?
Find us on Slack: https://pressioteam.slack.com or open an issue on [github](https://github.com/Pressio/pressio4py).
