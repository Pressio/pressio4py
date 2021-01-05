
# Python Bindings for Pressio

*Leading-edge projection-based reduced order models (\proms) for
dynamical systems in science and engineering.*

You landed on the documentation of the Python bindings library!

If this is a mistake, please go back to the [project website](https://pressio.github.io/).


# Get Started

Should be just a couple steps:

@m_class{m-block m-warning}

@par
```bash
export CXX=<path-to-your-C++-compiler>
git clone git@github.com:Pressio/pressio4py.git
pip install ./pressio4py
```

You can then run the regression tests:
```bash
cd pressio4py
pytest -s
```

Or look at the demos:
```bash
cd pressio4py/demos
python3 ./<demo-subdir-name>/main.py
```

You can find descriptions of the demos [here](./md_pages_demos_demo1.html)
and of the tutorials [here](./md_pages_tutorials_tutorial1.html)--- both in progress.

<!-- read the [building/installation process](./md_pages_getstarted_build_and_install.html)>
<!-- Untill we start filling the tutorials and examples, you can peek at the [test subdirectory](https://github.com/Pressio/pressio/tree/master/tests/rom/burgers1d) of the C++ library. -->


## License and Citation
The full license is available [here](https://pressio.github.io/various/license/).

We are working on publishing this: you can find our arXiv preprint at: https://arxiv.org/abs/2003.07798

## Questions?
Find us on Slack: https://pressioteam.slack.com or open an issue on [github](https://github.com/Pressio/pressio4py).
