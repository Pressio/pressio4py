
#ifndef PRESSIO4PY_PYBINDINGS_LINEAR_DECODER_HPP_
#define PRESSIO4PY_PYBINDINGS_LINEAR_DECODER_HPP_

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>

#include "ROM_BASIC"
#include "types.hpp"

template <typename types>
struct LinearDecoderBinder{
  using ops_t		= typename types::ops_t;
  using decoder_jac_t	= typename types::py_arr;
  using decoder_t	= ::pressio::rom::PyLinearDecoder<decoder_jac_t, ops_t>;

  static void doBind(pybind11::module & m){
    pybind11::class_<decoder_t>(m, "LinearDecoder")
      .def(pybind11::init< const decoder_jac_t &, pybind11::object &>());
  }
};

PYBIND11_MODULE(pressio4py, m) {
  using mytypes = MyTypes;
  LinearDecoderBinder<mytypes>::doBind(m);
}

#endif
