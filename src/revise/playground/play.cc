
#ifndef PRESSIO4PY_PYBINDINGS_TESTING_HPP_
#define PRESSIO4PY_PYBINDINGS_TESTING_HPP_

#include "pressio_containers.hpp"
#include "types.hpp"


namespace
{

template <typename T>
void wf(T & o){
  std::cout << o.size() << std::endl;
  // auto sp = ::pressio::containers::span(o, 2, 3);
  // std::cout << sp.extent(0) << std::endl;
  // T b1(10);

  // T b(o[pybind11::slice(2, 4, 1)]); // works
  // std::cout << b.size() << " " << b.at(0) << std::endl;

  std::cout << "o ad: " << o.data() << std::endl;

  pybind11::array_t<double> b(o[pybind11::slice(0, 4, 1)]);
  std::cout << b.data() << " " << b.size() << " " << b.at(0) << std::endl;
}
}

PYBIND11_MODULE(pressio4pyTesting, m) {

  using mytypes		= DefaultGalerkinTypes;
  using scalar_t	= typename mytypes::scalar_t;
  using fom_t		= typename mytypes::fom_t;
  using ops_t		= typename mytypes::ops_t;
  using rom_state_t	= typename mytypes::rom_state_t;

  //m.def("span", &::pressio::containers::span<rom_state_t, int, int>, "Span view");
  m.def("wf", &wf<rom_state_t>, "MyWf");
}
#endif
