
'''
see this for why this file exists and is done this way
https://stackoverflow.com/questions/47599162/pybind11-how-to-package-c-and-python-code-into-a-single-package?rq=1
'''
try:
  from ._p4pyimpl.ode import *
except ImportError:
  raise ImportError("Unable to import ode from _p4pyimpl")
