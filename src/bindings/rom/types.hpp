
#ifndef PRESSIO4PY_PYBINDINGS_TYPES_HPP_
#define PRESSIO4PY_PYBINDINGS_TYPES_HPP_

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>

struct MyTypes{
  using scalar_t	= double;
  using py_arr		= pybind11::array_t<scalar_t, pybind11::array::c_style>;
  using fom_t		= pybind11::object;
  using ops_t		= pybind11::object;
  using rom_state_t	= py_arr;
  using step_t		= int;
};

#endif
