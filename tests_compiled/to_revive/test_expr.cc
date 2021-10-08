
#include "types.hpp"
#include "pressio/type_traits.hpp"
#include "pressio/expressions.hpp"
#include "pressio/ops.hpp"

namespace
{

bool test_span(pressio4py::py_f_arr a)
{
  // span starting from index=2
  auto s = pressio::span(a, 2, 5);
  if (s.extent(0)!=5) return false;

  if (s(0)!=2.) return false;
  if (s(1)!=3.) return false;
  if (s(2)!=4.) return false;
  if (s(3)!=5.) return false;
  if (s(4)!=6.) return false;

  s(0)=1.5;
  s(1)=2.5;
  s(2)=3.5;
  if (a(2)!=1.5) return false;
  if (a(3)!=2.5) return false;
  if (a(4)!=3.5) return false;

  return true;
}

bool test_subspan1(pressio4py::py_f_arr A)
{
  const auto sspan = pressio::subspan(A,
				      std::make_pair(1,3),
				      std::make_pair(2,4) );
  if (sspan.extent(0) != 2 ) return false;
  if (sspan.extent(1) != 2 ) return false;
  if (sspan(0,0) != 7.)  return false;
  if (sspan(1,0) != 11.)  return false;
  if (sspan(0,1) != 8.)  return false;
  if (sspan(1,1) != 12.)  return false;
  return true;
}

bool test_subspan2(pressio4py::py_f_arr A)
{
  auto sspan = pressio::subspan(A, std::make_pair(2,5), std::make_pair(2,4) );
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

  // original should be changed too
  if (A(0,0) != 1.) return false;
  if (A(0,1) != 2.) return false;
  if (A(0,2) != 3.) return false;
  if (A(0,3) != 4) return false;

  if (A(1,0) != 5.) return false;
  if (A(1,1) != 6.) return false;
  if (A(1,2) != 7.) return false;
  if (A(1,3) != 8) return false;

  if (A(2,0) != 9.) return false;
  if (A(2,1) != 10.) return false;
  if (A(2,2) != 1.2) return false;
  if (A(2,3) != 1.2) return false;

  if (A(3,0) != 13.) return false;
  if (A(3,1) != 14.) return false;
  if (A(3,2) != 1.2) return false;
  if (A(3,3) != 1.2) return false;

  if (A(4,0) != 17.) return false;
  if (A(4,1) != 18.) return false;
  if (A(4,2) != 1.2) return false;
  if (A(4,3) != 2.2) return false;

  if (A(5,0) != 21.) return false;
  if (A(5,1) != 22.) return false;
  if (A(5,2) != 23.) return false;
  if (A(5,3) != 24) return false;

  return true;
}

bool test_diag1(pressio4py::py_f_arr A)
{
  const auto dd = pressio::diag(A);
  if (dd.extent(0) != 4 ) return false;
  if (dd(0) != 1.)  return false;
  if (dd(1) != 6.)  return false;
  if (dd(2) != 11.)  return false;
  if (dd(3) != 16.)  return false;
  return true;
}

bool test_diag2(pressio4py::py_f_arr A)
{
  auto dd = pressio::diag(A);
  if (dd.extent(0) != 4 ) return false;
  if (dd(0) != 1.)  return false;
  if (dd(1) != 6.)  return false;
  if (dd(2) != 11.)  return false;
  if (dd(3) != 16.)  return false;

  // change values
  dd(0) = 1.2;
  dd(1) = 1.2;
  dd(2) = 1.2;
  dd(3) = 1.2;

  // original should be changed too
  if (A(0,0) != 1.2) return false;
  if (A(0,1) != 2.) return false;
  if (A(0,2) != 3.) return false;
  if (A(0,3) != 4) return false;

  if (A(1,0) != 5.) return false;
  if (A(1,1) != 1.2) return false;
  if (A(1,2) != 7.) return false;
  if (A(1,3) != 8) return false;

  if (A(2,0) != 9.) return false;
  if (A(2,1) != 10.) return false;
  if (A(2,2) != 1.2) return false;
  if (A(2,3) != 12.) return false;

  if (A(3,0) != 13.) return false;
  if (A(3,1) != 14.) return false;
  if (A(3,2) != 15.) return false;
  if (A(3,3) != 1.2) return false;

  return true;
}

}//anonym namespace

PYBIND11_MODULE(test_expr_module, m) {
  m.def("span", &test_span);
  m.def("subspan1", &test_subspan1);
  m.def("subspan2", &test_subspan2);
  m.def("diag1", &test_diag1);
  m.def("diag2", &test_diag2);
}
