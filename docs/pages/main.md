
# Python Bindings for Pressio

*Leading-edge projection-based reduced order models (\proms) for
dynamical systems in science and engineering.*

You landed on the documentation of the pressio Python bindings library!

If this is a mistake, please go back to the [project website](https://pressio.github.io/).


## Install pressio4py

@m_class{m-block m-warning}

@par
```bash
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


## Where to go from here?

### Learn how to interface your app

To enable pressio4py to communicate with your app, you might
have to prepare some glue code in the form of an adapter class
as described [here](./md_pages_prepare_your_app.html).
Note that this only needs to be done once: *the same interface
class can then be used to run all the ROMs in pressio4py*.

### Explore the tutorials and demos
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
