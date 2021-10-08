
#include "pressio_containers.hpp"
#include "types.hpp"

namespace
{

bool test1(pressio4py::py_f_arr A)
{
  using in_t = pressio4py::py_f_arr;
  using w_t = ::pressio::containers::Tensor<2,in_t>;
  w_t Aw(A);
  static_assert
  (::pressio::containers::predicates::is_tensor_wrapper<w_t>::value 
    and w_t::traits::rank==2, "");
  static_assert
  (::pressio::containers::predicates::is_rank2_tensor_wrapper_pybind<w_t>::value, "");

  const auto sspan = pressio::containers::subspan(Aw, std::make_pair(1,3), std::make_pair(2,4) );
  if (sspan.extent(0) != 2 ) return false;
  if (sspan.extent(1) != 2 ) return false;
  if (sspan(0,0) != 7.)  return false;
  if (sspan(1,0) != 11.)  return false;
  if (sspan(0,1) != 8.)  return false;
  if (sspan(1,1) != 12.)  return false;
  return true;
}

bool test2(pressio4py::py_f_arr A)
{
  using in_t = pressio4py::py_f_arr;
  using w_t = ::pressio::containers::Tensor<2,in_t>;
  w_t Aw(A);
  static_assert
  (::pressio::containers::predicates::is_tensor_wrapper<w_t>::value 
    and w_t::traits::rank==2, "");
  static_assert
  (::pressio::containers::predicates::is_rank2_tensor_wrapper_pybind<w_t>::value, "");

  auto sspan = pressio::containers::subspan(Aw, std::make_pair(2,5), std::make_pair(2,4) );
  if (sspan.extent(0) != 3 ) return false;
  if (sspan.extent(1) != 2 ) return false;
  if (sspan(0,0) != 11.)  return false;
  if (sspan(1,0) != 15.)  return false;
  if (sspan(2,0) != 19.)  return false;
  if (sspan(0,1) != 12.)  return false;
  if (sspan(1,1) != 16.)  return false;
  if (sspan(2,1) != 20.)  return false;

  // change values
  sspan(0,0) = 1.2;
  sspan(1,0) = 1.2;
  sspan(2,0) = 1.2;
  sspan(0,1) = 1.2;
  sspan(1,1) = 1.2;
  sspan(2,1) = 2.2;

  // original Aw should be changed too
  
  if (Aw(0,0) != 1.) return false;
  if (Aw(0,1) != 2.) return false;
  if (Aw(0,2) != 3.) return false;
  if (Aw(0,3) != 4) return false;
  
  if (Aw(1,0) != 5.) return false;
  if (Aw(1,1) != 6.) return false;
  if (Aw(1,2) != 7.) return false;
  if (Aw(1,3) != 8) return false;
  
  if (Aw(2,0) != 9.) return false;
  if (Aw(2,1) != 10.) return false;
  if (Aw(2,2) != 1.2) return false;
  if (Aw(2,3) != 1.2) return false;
  
  if (Aw(3,0) != 13.) return false;
  if (Aw(3,1) != 14.) return false;
  if (Aw(3,2) != 1.2) return false;
  if (Aw(3,3) != 1.2) return false;
  
  if (Aw(4,0) != 17.) return false;
  if (Aw(4,1) != 18.) return false;
  if (Aw(4,2) != 1.2) return false;
  if (Aw(4,3) != 2.2) return false;
  
  if (Aw(5,0) != 21.) return false;
  if (Aw(5,1) != 22.) return false;
  if (Aw(5,2) != 23.) return false;
  if (Aw(5,3) != 24) return false;

  return true;
}

}//anonym namespace

PYBIND11_MODULE(test_subspan_module, m) {
  m.def("subspan1", &test1);
  m.def("subspan2", &test2);
}
