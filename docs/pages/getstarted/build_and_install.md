
# Building Pressio4py


@m_class{m-block m-info}

@par What is this page about?
This page describes the building steps for `pressio4py`.
By the end, you should be able to clone pressio4py, build it,
and use the library in Python.


## Prerequisites
* SSH keys setup with github (if you are working behind a firewall, set the proper proxies)
* C, C++ (with support for C++14) compiler;
* CMake >= 3.11.0;
* Bash >= 3.2.57.
* Python (>=3.6), with NumPy, SciPy, Numba, and Pytest


## In pratice, to build pressio4py follow these steps

### 1. Prep

Create (or choose) a directory where you want to clone all repos, e,g.:

```bash
mkdir $HOME/pressio_repos
```

And to make things easier, create environment variables:

```bash
export PRESSIO_REPOS=$HOME/pressio_repos
export PRESSIO_BUILDS=$HOME/pressio_builds
export CC=<path-to-your-C-compiler>
export CXX=<path-to-your-CXX-compiler>
```


### 2. Cloning

Clone the needed repos as follows:

```bash
cd ${PRESSIO_REPOS}

git clone git@github.com:Pressio/pressio.git
git clone git@github.com:Pressio/pressio-builder.git
git clone git@github.com:Pressio/pressio4py.git
```


### 3. Install TPLs

Run the following command:

```bash
cd ${PRESSIO_REPOS}/pressio-builder
bash ./main_tpls.sh -dryrun=no -build-mode=Debug \
     -target-dir=${PRESSIO_BUILDS} -tpls=pybind11
```
This step should create two subdirectories:
```bash
${PRESSIO_BUILDS}/pybind11/{install, build}
```


### 4. Install pressio

```bash
cd ${PRESSIO_REPOS}/pressio-builder
./main_pressio.sh -dryrun=no \
  -pressio-src=${PRESSIO_REPOS}/pressio \
  -target-dir=${PRESSIO_BUILDS} \
  -cmake-generator-name=default_pybind \
  -pybind11-path=${PRESSIO_BUILDS}/pybind11/install
```


### 5. Time to build pressio4py

```bash
cd ${PRESSIO_BUILDS}

bdirname=pressio4py-build
rm -rf ${bdirname} && mkdir ${bdirname} && cd ${bdirname}
cmake -DCMAKE_VERBOSE_MAKEFILE:BOOL=TRUE \
      -DCMAKE_INSTALL_PREFIX=${PRESSIO_BUILDS}/pressio4py-install \
      -DCMAKE_CXX_COMPILER=${CXX} \
      -DCMAKE_BUILD_TYPE=Release \
      -DPRESSIO_INCLUDE_DIR=${PRESSIO_BUILDS}/pressio/install/include \
      -DPYBIND11_DIR=${PRESSIO_BUILDS}/pybind11/install \
      ${PRESSIO_REPOS}/pressio4py
make -j4
make install
cd ..
```
This creates the Python library inside
`${PRESSIO_BUILDS}/pressio4py-install` to load.


### 6. Testing

After building, you can do:
```bash
cd ${PRESSIO_BUILDS}/pressio4py-build
pytest -s
```
where the `-s` flag tells Python to flush all output to terminal.
