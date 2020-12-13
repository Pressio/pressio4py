# -*- coding: utf-8 -*-
import os
import sys
import subprocess

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext

# A CMakeExtension needs a sourcedir instead of a file list.
class CMakeExtension(Extension):
  def __init__(self, name, sourcedir=""):
    Extension.__init__(self, name, sources=[])
    self.sourcedir = os.path.abspath(sourcedir)

class CMakeBuild(build_ext):
  def build_extension(self, ext):
    extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))

    if not extdir.endswith(os.path.sep): extdir += os.path.sep

    pressioIncludeDir = ext.sourcedir + "/pressio/packages"
    print("pressioIncludeDir = ", pressioIncludeDir)

    cfg = "Debug" if self.debug else "Release"

    # CMake lets you override the generator - we need to check this.
    # Can be set with Conda-Build, for example.
    cmake_generator = os.environ.get("CMAKE_GENERATOR", "")

    # Set Python_EXECUTABLE instead if you use PYBIND11_FINDPYTHON
    # EXAMPLE_VERSION_INFO shows you how to pass a value into the C++ code
    # from Python.
    cmake_args = [
      "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={}".format(extdir),
      "-DPYTHON_EXECUTABLE={}".format(sys.executable),
      "-DCMAKE_BUILD_TYPE={}".format(cfg),
      "-DPRESSIO_INCLUDE_DIR={}".format(pressioIncludeDir),
    ]
    build_args = []

    if "CXX" not in os.environ:
      msg = "CXX env var missing, needs to point to your target C++ compiler"
      raise RuntimeError(msg)

    # Set CMAKE_BUILD_PARALLEL_LEVEL to control the parallel build level
    # across all generators.
    if "CMAKE_BUILD_PARALLEL_LEVEL" not in os.environ:
      # self.parallel is a Python 3 only way to set parallel jobs by hand
      # using -j in the build_ext call, not supported by pip or PyPA-build.
      if hasattr(self, "parallel") and self.parallel:
        # CMake 3.12+ only.
        build_args += ["-j{}".format(self.parallel)]

    if not os.path.exists(self.build_temp):
      os.makedirs(self.build_temp)

    subprocess.check_call(
      ["cmake", ext.sourcedir] + cmake_args, cwd=self.build_temp
    )
    subprocess.check_call(
      ["cmake", "--build", "."] + build_args, cwd=self.build_temp
    )

setup(
  name="pressio4py",
  version="0.5.2",
  author="Francesco Rizzi",
  author_email="fnrizzi@sandia.gov",
  description="pressio4py: projection-based model reduction for Python",
  url="https://github.com/Pressio/pressio4py",
  long_description="Projection-based model reduction for (large-scale) linear and nonlinear dynamical systems",
  ext_modules=[CMakeExtension("pressio4py")],
  cmdclass={"build_ext": CMakeBuild},
  install_requires=["numpy", "scipy", "pybind11", "numba"],
  zip_safe=False,
)
