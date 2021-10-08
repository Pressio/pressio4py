
# Python Bindings for Pressio


@m_class{m-frame m-default}

@parblock
*Leading-edge reduced order models (ROMs) for dynamical systems in science and engineering.*

This is the documentation of the [Python library](https://github.com/Pressio/pressio4py), one component of the [Pressio ecosystem](https://pressio.github.io/).
@endparblock

## Start with why

Model reduction is a broad and very active field.
Many methods exist, but there is no such thing as "one method to rule them all".
We believe that evaluating the quality of a reduced model requires
accounting for several factors, e.g., the reduction in degrees of freedom,
training cost, evaluation cost, robustness, simplicity, predictive accuracy, etc.
There is no single metric to rely on; it is always a tradeoff.

We believe that there is a lot to explore in this field
both in terms of new research directions as well as assessing
robustness of current state-of-the-art methods.
There is no better way than a Python framework to incentivize and
foster work to impact this field. Working towards this goal, pressio4py
is our contribution to research novel fundamental ideas
on model reduction as well as test state-of-the-art methods
on problems of arbitrary complexity. Python is the perfect language
to do so because it benefits from a large community of developers
and has become the de-facto choice for machine learning.
This offers an ideal framework to explore and merge ideas from different fields.


## Components

| Name                                                 | Description/Content                                                                              | Links                                                                                                                                                                                                                                                                                                                                                                               | Module(s)                         |
| ------------------                                   | ---------------                                                                                  | ------------                                                                                                                                                                                                                                                                                                                                                                        |                                   |
| @m_span{m-text m-success}logger@m_endspan             | (dis)enable pressio logging                                                | <br/>[Doc Page](md_pages_components_logger.html)                                                                                                                                                                                                                                                      | `pressio4py.logger`                                                                                                                                                              |
| @m_span{m-text m-success}solvers_nonlinear@m_endspan | <br/> general info <br/> Newton-Raphson <br/> Gauss-Newton <br/> Levenberg-Marquardt <br/>       | <br/> [Doc Page](md_pages_components_nonlinsolvers_general.html) <br/> [Doc Page](md_pages_components_nonlinsolvers_nr.html) <br/> [Doc Page](md_pages_components_nonlinsolvers_gn.html) <br/> [Doc Page](md_pages_components_nonlinsolvers_lm.html)                  | `pressio4py.solvers` |
| @m_span{m-text m-success}ode@m_endspan               | <br/> explicit steppers <br/>implicit steppers <br/> advancers <br/>                             | <br/> [Doc Page](md_pages_components_ode_steppers_explicit.html)<br/> [Doc Page](md_pages_components_ode_steppers_implicit.html) <br/>[Doc Page](md_pages_components_ode_advance.html)                                                                                                                                                                               | `pressio4py.ode`                  |
| @m_span{m-text m-success}rom@m_endspan               | <br/>general info <br/> decoder <br/> Galerkin<br/> least-squares Petrov-Galerkin (LSPG): steady<br/> least-squares Petrov-Galerkin (LSPG): unsteady<br/> Windowed least-squares (WLS)<br/> | <br/>[Doc Page](md_pages_components_rom_general.html) <br/>[Doc Page](md_pages_components_rom_decoder.html) <br/> [Doc Page](md_pages_components_rom_galerkin.html) <br/> [Doc Page](md_pages_components_rom_lspg_steady.html) <br/> [Doc Page](md_pages_components_rom_lspg_unsteady.html) <br/>  [Doc Page](md_pages_components_rom_wls.html) <br/> | `pressio4py.rom`                  |


Note that we intentially keep pressio4py limited in scope for now.
We don't provide bindings to all the functionalities
in the [pressio C++ library](https://pressio.github.io/pressio/html/index.html)
but only bind those for model reduction and the auxiliary ones.



## Installation

@m_class{m-block m-primary}

@par
```bash
# the C++ compiler must support C++14
export CXX=<path-to-your-C++-compiler>

git clone --recursive git@github.com:Pressio/pressio4py.git
# or https://github.com/Pressio/pressio4py.git

pip install ./pressio4py

# verify installation by running our test suite:
cd pressio4py
pytest -s
```
@endparblock

@m_class{m-note m-info}

@parblock
If you get an import error, make sure the version of `pytest` you are
using is compatible with the `pip` command you used to install.
For compatibility, the Python commands must be from the **same** distribution.
@endparblock

<!-- ## Explore the tutorials and demos -->
<!-- You can find descriptions of the demos [here](./md_pages_demos_demo1.html) -->
<!-- and of the tutorials [here](./md_pages_tutorials_tutorial1.html)---we will progressively add more. -->
<!-- ```bash -->
<!-- cd pressio4py/demos -->
<!-- python3 ./<demo-subdir-name>/main.py -->
<!-- ``` -->


<!-- read the [building/installation process](./md_pages_getstarted_build_and_install.html)>
<!-- Untill we start filling the tutorials and examples, you can peek at the [test subdirectory](https://github.com/Pressio/pressio/tree/master/tests/rom/burgers1d) of the C++ library. -->

<br/>

## License and Citation
The full license is available [here](https://pressio.github.io/various/license/).

We are working on publishing this: you can find our arXiv preprint at: https://arxiv.org/abs/2003.07798

## Questions?
Find us on Slack: https://pressioteam.slack.com or open an issue on [github](https://github.com/Pressio/pressio4py).
