
#include "types.hpp"
#include "pressio/ops.hpp"

namespace
{

using pressio4py::py_f_arr;
using pressio4py::py_c_arr;

bool myextent(py_f_arr a){
  if (a.ndim()==1){
    return (::pressio::ops::extent(a,0) == 5);
  }else{
    return (::pressio::ops::extent(a,0) == 5) &&
      (::pressio::ops::extent(a,1) == 6);
  }
}

py_f_arr myclone(const py_f_arr a){
  return ::pressio::ops::clone(a);
}

void deepcopy(py_f_arr dest, py_f_arr src){
  ::pressio::ops::deep_copy(dest, src);
}

void myabs(py_f_arr b, py_f_arr a){
  ::pressio::ops::abs(b,a);
}

void scale(py_f_arr a, double val){
  ::pressio::ops::scale(a, val);
}

void fill(py_f_arr a, double val){
  ::pressio::ops::fill(a, val);
}

void set_zero(py_f_arr a){
  ::pressio::ops::set_zero(a);
}

auto max(const py_f_arr a){
  return ::pressio::ops::max(a);
}

auto min(const py_f_arr a){
  return ::pressio::ops::min(a);
}

auto norm1(const py_f_arr a){
  return ::pressio::ops::norm1(a);
}

auto norm2(const py_f_arr a){
  return ::pressio::ops::norm2(a);
}

auto dot(py_f_arr a, py_f_arr b){
  return ::pressio::ops::dot(a, b);
}

}//anonym namespace

PYBIND11_MODULE(test_ops_rank1_array_module, m)
{
  m.def("extent", &myextent);
  m.def("clone", &myclone);
  m.def("deep_copy", &deepcopy);
  m.def("abs", &myabs);
  m.def("scale", &scale);
  m.def("fill", &fill);
  m.def("set_zero", &set_zero);
  m.def("max", &max);
  m.def("min", &min);
  m.def("norm1", &norm1);
  m.def("norm2", &norm2);
  m.def("dot", &dot);
}
