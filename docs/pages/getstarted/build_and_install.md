
# Building Pressio4py


@m_class{m-block m-info}

@par What does this page describe?
This page describes the end-to-end building process for `pressio4py`.
By the end, it should be clear how to clone pressio4py, get all dependencies ,
build pressio4py and use the library in Python.


@m_class{m-block m-warning}

@par We will soon enable installation via pip, stay tuned!


## Prerequisites
* SSH keys setup with github (if you are working behind a firewall, set the proper proxies);
* C++ compiler (with support for C++14 standard);
* CMake >= 3.11.0;
* Bash >= 3.2.57.
* Python (>=3.6), with NumPy, SciPy, Numba, Pytest and sklearn (needed to run some of the demos).


## Pratical steps

### 1. Prep

Create a working director where you want to clone all repos and do the builds, e,g.:

```bash
export WORKDIR=$HOME/myPressioTest
mkdir -p ${WORKDIR}/sources  # creates workdir and subdir for all sources
```

To make things easier, create environment variables for the compilers:

```bash
export CC=<path-to-your-C-compiler>
export CXX=<path-to-your-CXX-compiler>
```

### 2. Cloning

```bash
cd ${WORKDIR}/sources
git clone git@github.com:Pressio/pressio.git
git clone git@github.com:Pressio/pressio-builder.git
git clone git@github.com:Pressio/pressio4py.git
```
By default, the above commands will clone the *master* branch of each repo.
This is because the master branches of the three repos are the stable ones,
and compatible with one another.
If you want specific versions, you can just checkout the tag you want.
If you do so, make sure you checkout the same tag for all three repos.


### 3. Install Dependencies

Currently, pressio and pybind11 are the dependencies required by `pressio4py`.
To get and install the TPLs, run the following:

```bash
cd ${WORKDIR}/sources/pressio-builder

# first, get pybind11
bash ./main_tpls.sh -dryrun=no -build-mode=Debug \
     -target-dir=${WORKDIR} -tpls=pybind11

# second, get pressio
./main_pressio.sh -dryrun=no \
  -pressio-src=${WORKDIR}/sources/pressio \
  -target-dir=${WORKDIR} \
  -cmake-generator-name=default_pybind \
  -pybind11-path=${WORKDIR}/pybind11/install
```
Inside `${WORKDIR}`, you should see the following structure:
```txt
➤ tree -d -L 2
.
├── pressio
│   ├── build
│   └── install
├── pybind11
│   ├── build
│   ├── install
│   └── pybind11
└── sources
    ├── pressio
    ├── pressio-builder
    └── pressio4py
```

### 4. Build pressio4py

```bash
cd ${WORKDIR}

bdirname=pressio4py-build
rm -rf ${bdirname} && mkdir ${bdirname} && cd ${bdirname}
cmake -DCMAKE_VERBOSE_MAKEFILE:BOOL=TRUE \
      -DCMAKE_INSTALL_PREFIX=${WORKDIR}/pressio4py-install \
      -DCMAKE_CXX_COMPILER=${CXX} \
      -DCMAKE_BUILD_TYPE=Release \
      -DPRESSIO_INCLUDE_DIR=${WORKDIR}/pressio/install/include \
      -DPYBIND11_DIR=${WORKDIR}/pybind11/install \
      ${WORKDIR}/sources/pressio4py
make -j4
make install
cd ..
```
This should create the Python library inside `${WORKDIR}/pressio4py-install`
which you can import in Python.

### 5. Running the tests or demos

To run all tests, after building:
```bash
cd ${WORKDIR}/pressio4py-build
pytest -s  #-s flushes all output to terminal.
```
Or you can find demos inside `${WORKDIR}/pressio4py-build/demos`.
