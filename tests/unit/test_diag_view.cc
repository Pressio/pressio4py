
#include "pressio_containers.hpp"
#include "types.hpp"

namespace
{

bool test1(typename pressio4py::CommonTypes::py_f_arr A)
{
  using in_t = typename pressio4py::CommonTypes::py_f_arr;
  using mat_t = ::pressio::containers::DenseMatrix<in_t>;
  mat_t Aw(A);
  const auto d = pressio::containers::diag(Aw);
  if (d(0)!=1.) return false;
  if (d(1)!=2.) return false;
  if (d(2)!=3.) return false;
  if (d(3)!=4.) return false;
  return true;
}

bool test2(typename pressio4py::CommonTypes::py_f_arr A)
{
  using in_t = typename pressio4py::CommonTypes::py_f_arr;
  using mat_t = ::pressio::containers::DenseMatrix<in_t>;
  mat_t Aw(A);
  if (Aw(0,0)!=1.) return false;
  if (Aw(1,1)!=2.) return false;
  if (Aw(2,2)!=3.) return false;
  if (Aw(3,3)!=4.) return false;

  // view diagonal and change it
  auto d = pressio::containers::diag(Aw);
  d(0)=1.5;
  d(1)=2.5;
  d(2)=3.5;
  d(3)=4.5;

  // Aw should be changed too
  if (Aw(0,0)!=1.5) return false;
  if (Aw(1,1)!=2.5) return false;
  if (Aw(2,2)!=3.5) return false;
  if (Aw(3,3)!=4.5) return false;
  return true;
}

}//anonym namespace

PYBIND11_MODULE(test_diag_view_module, m) {
  m.def("diagViewConstruct", &test1);
  m.def("diagViewModify", &test2);
}
