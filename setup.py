#!/usr/bin/env python
# Author:  Francesco Rizzi
# Contact: fnrizzi@sandia.gov, francesco.rizzi@ng-analytics.com

import os
import sys
import subprocess

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext

topdir = os.path.abspath(os.path.dirname(__file__))

def description():
    with open(os.path.join(topdir, 'DESCRIPTION.rst')) as f:
        return f.read()

# A CMakeExtension needs a sourcedir instead of a file list.
class CMakeExtension(Extension):
  def __init__(self, name, sourcedir=""):
    Extension.__init__(self, name, sources=[])
    self.sourcedir = os.path.abspath(sourcedir)

class CMakeBuild(build_ext):
  def build_extension(self, ext):
    extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))

    if not extdir.endswith(os.path.sep): extdir += os.path.sep

    if not os.path.exists(self.build_temp): os.makedirs(self.build_temp)
    print("self.build_temp ", self.build_temp)

    # CMake lets you override the generator - we need to check this.
    # Can be set with Conda-Build, for example.
    cmake_generator = os.environ.get("CMAKE_GENERATOR", "")

    #------------------------
    # --- pressio ---
    #------------------------
    pressioSrcDir   = ext.sourcedir + "/pressio"
    pressioBuildDir = self.build_temp+"/pressio/build"
    pressioPfxDir   = "../install"
    cmake_args = [
      "-DPRESSIO_ENABLE_TPL_EIGEN=OFF",
      "-DPRESSIO_ENABLE_TPL_PYBIND11=ON",
      "-DCMAKE_INSTALL_PREFIX={}".format(pressioPfxDir),
    ]
    if not os.path.exists(pressioBuildDir): os.makedirs(pressioBuildDir)
    subprocess.check_call(
      ["cmake", pressioSrcDir] + cmake_args, cwd=pressioBuildDir
    )
    subprocess.check_call(
      ["make", "install"], cwd=pressioBuildDir
    )
    pressioIncludeDir = self.build_temp + "/pressio/install/include"
    print("pressioIncludeDir = ", pressioIncludeDir)

    #------------------------
    # --- pressio4py ---
    #------------------------
    cfg = "Debug" if self.debug else "Release"

    # Set Python_EXECUTABLE instead if you use PYBIND11_FINDPYTHON
    # EXAMPLE_VERSION_INFO shows you how to pass a value into the C++ code
    # from Python.
    cmake_args = [
      "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={}".format(extdir),
      "-DPYTHON_EXECUTABLE={}".format(sys.executable),
      "-DCMAKE_BUILD_TYPE={}".format(cfg),
      "-DPRESSIO_INCLUDE_DIR={}".format(pressioIncludeDir),
      "-DVERSION_INFO={}".format(self.distribution.get_version()),
    ]
    build_args = []

    if "CXX" not in os.environ:
      msg = "CXX env var missing, needs to point to your target C++ compiler"
      raise RuntimeError(msg)

    # Set CMAKE_BUILD_PARALLEL_LEVEL to control the parallel build level
    # across all generators.
    if "CMAKE_BUILD_PARALLEL_LEVEL" not in os.environ:
      # for now, set parallel level to 4
      build_args += ["-j4"]
      # # self.parallel is a Python 3 only way to set parallel jobs by hand
      # # using -j in the build_ext call, not supported by pip or PyPA-build.
      # if hasattr(self, "parallel") and self.parallel:
      #   # CMake 3.12+ only.
      #   build_args += ["-j{}".format(self.parallel)]

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
  version="0.6.1rc4",
  author="Francesco Rizzi",
  author_email="fnrizzi@sandia.gov",
  description="pressio4py: projection-based model reduction for Python",
  #
  project_urls={
    'Documentation': 'https://pressio.github.io/pressio4py/html/index.html',
    'Source': 'https://github.com/Pressio/pressio4py'
  },
  #
  long_description=description(),
  ext_modules=[CMakeExtension("pressio4py")],
  cmdclass={"build_ext": CMakeBuild},
  install_requires=["numpy", "scipy", "matplotlib", "sklearn"],
  zip_safe=False,
  #
  python_requires='>=3',
  classifiers=[
    "License :: OSI Approved :: BSD License",
    "Operating System :: Unix",
    "Environment :: MacOS X",
    "Programming Language :: C++",
    "Programming Language :: Python :: 3 :: Only",
    "Topic :: Scientific/Engineering",
    "Topic :: Scientific/Engineering :: Mathematics",
    "Topic :: Scientific/Engineering :: Physics",
    "Topic :: Software Development :: Libraries",
    "Development Status :: 4 - Beta"
  ],
  keywords=["model reduction", "scientific computing", "dense linear algebra", "HPC"],
)
