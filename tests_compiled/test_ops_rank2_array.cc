
#include "types.hpp"
#include "pressio/ops.hpp"

namespace
{

using pressio4py::py_f_arr;
using pressio4py::py_c_arr;

bool myextent_f(py_f_arr a){
  if (a.ndim()==1){
    return (::pressio::ops::extent(a,0) == 5);
  }else{
    return (::pressio::ops::extent(a,0) == 5) &&
      (::pressio::ops::extent(a,1) == 6);
  }
}

bool myextent_c(py_c_arr a){
  if (a.ndim()==1){
    return (::pressio::ops::extent(a,0) == 5);
  }else{
    return (::pressio::ops::extent(a,0) == 5) &&
      (::pressio::ops::extent(a,1) == 6);
  }
}

py_f_arr myclone_f(const py_f_arr a){
  return ::pressio::ops::clone(a);
}

py_c_arr myclone_c(const py_c_arr a){
  return ::pressio::ops::clone(a);
}

void deepcopy_f(py_f_arr dest, py_f_arr src){
  ::pressio::ops::deep_copy(dest, src);
}

void deepcopy_c(py_c_arr dest, py_c_arr src){
  ::pressio::ops::deep_copy(dest, src);
}

void deepcopy_fc(py_f_arr dest, py_c_arr src){
  ::pressio::ops::deep_copy(dest, src);
}

void deepcopy_cf(py_c_arr dest, py_f_arr src){
  ::pressio::ops::deep_copy(dest, src);
}

void scale_f(py_f_arr a, double val){
  ::pressio::ops::scale(a, val);
}

void scale_c(py_c_arr a, double val){
  ::pressio::ops::scale(a, val);
}

void fill_f(py_f_arr a, double val){
  ::pressio::ops::fill(a, val);
}

void fill_c(py_c_arr a, double val){
  ::pressio::ops::fill(a, val);
}

void set_zero_f(py_f_arr a){
  ::pressio::ops::set_zero(a);
}

void set_zero_c(py_c_arr a){
  ::pressio::ops::set_zero(a);
}

void add_to_diagonal_f(py_f_arr a, double val){
  ::pressio::ops::add_to_diagonal(a, val);
}

void add_to_diagonal_c(py_c_arr a, double val){
  ::pressio::ops::add_to_diagonal(a, val);
}

}//anonym namespace

PYBIND11_MODULE(test_ops_rank2_array_module, m)
{
  m.def("extent", &myextent_f);
  m.def("extent", &myextent_c);
  m.def("clone", &myclone_f);
  m.def("clone", &myclone_c);
  m.def("deep_copy", &deepcopy_f);
  m.def("deep_copy", &deepcopy_c);
  m.def("deep_copy", &deepcopy_fc);
  m.def("deep_copy", &deepcopy_cf);
  m.def("scale", &scale_f);
  m.def("scale", &scale_c);
  m.def("fill", &fill_f);
  m.def("fill", &fill_c);
  m.def("set_zero", &set_zero_f);
  m.def("set_zero", &set_zero_c);
  m.def("add_to_diagonal", &add_to_diagonal_f);
  m.def("add_to_diagonal", &add_to_diagonal_c);
}
