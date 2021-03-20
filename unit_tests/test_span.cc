
#include "pressio_containers.hpp"
#include "types.hpp"

namespace
{

bool test1(pressio4py::py_f_arr A)
{
  using in_t = pressio4py::py_f_arr;
  using w_t = ::pressio::containers::Tensor<1,in_t>;
  w_t Aw(A);
  if (Aw.extent(0)!=8) return false;
  static_assert
  (::pressio::containers::predicates::is_tensor_wrapper<w_t>::value 
    and w_t::traits::rank==1, "");

  // span starting from index=2
  const auto s = pressio::containers::span(Aw, 2, 5);
  if (s.extent(0)!=5) return false;

  if (s(0)!=2.) return false;
  if (s(1)!=3.) return false;
  if (s(2)!=4.) return false;
  if (s(3)!=5.) return false;
  if (s(4)!=6.) return false;
  return true;
}

bool test2(pressio4py::py_f_arr A)
{
  using in_t = pressio4py::py_f_arr;
  using w_t = ::pressio::containers::Tensor<1,in_t>;
  w_t Aw(A);
  if (Aw(0)!=0.) return false;
  if (Aw(1)!=1.) return false;
  if (Aw(2)!=2.) return false;
  if (Aw(3)!=3.) return false;
  if (Aw(4)!=4.) return false;
  if (Aw(5)!=5.) return false;
  if (Aw(6)!=6.) return false;
  if (Aw(7)!=7.) return false;

  // span change it
  auto d = pressio::containers::span(Aw, 3, 3);
  d(0)=1.5;
  d(1)=2.5;
  d(2)=3.5;

  if (Aw(0)!=0.) return false;
  if (Aw(1)!=1.) return false;
  if (Aw(2)!=2.) return false;
  if (Aw(3)!=1.5) return false;
  if (Aw(4)!=2.5) return false;
  if (Aw(5)!=3.5) return false;
  if (Aw(6)!=6.) return false;
  if (Aw(7)!=7.) return false;
  return true;
}

}//anonym namespace

PYBIND11_MODULE(test_span_module, m) {
  m.def("spanConstruct", &test1);
  m.def("spanModify", &test2);
}
