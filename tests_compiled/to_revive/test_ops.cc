
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

void myabs_f(py_f_arr b, py_f_arr a){
  ::pressio::ops::abs(b,a);
}

void myabs_c(py_c_arr b, py_c_arr a){
  ::pressio::ops::abs(b,a);
}

void myabs_fc(py_f_arr b, py_c_arr a){
  ::pressio::ops::abs(b,a);
}

void myabs_cf(py_c_arr b, py_f_arr a){
  ::pressio::ops::abs(b,a);
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

auto max_f(const py_f_arr a){
  return ::pressio::ops::max(a);
}

auto max_c(const py_c_arr a){
  return ::pressio::ops::max(a);
}

auto min_f(const py_f_arr a){
  return ::pressio::ops::min(a);
}

auto min_c(const py_c_arr a){
  return ::pressio::ops::min(a);
}

auto norm1_f(const py_f_arr a){
  return ::pressio::ops::norm1(a);
}

auto norm1_c(const py_c_arr a){
  return ::pressio::ops::norm1(a);
}

auto norm2_f(const py_f_arr a){
  return ::pressio::ops::norm2(a);
}

auto norm2_c(const py_c_arr a){
  return ::pressio::ops::norm2(a);
}

auto dot_c(py_c_arr a, py_c_arr b){
  return ::pressio::ops::dot(a, b);
}

auto dot_f(py_f_arr a, py_f_arr b){
  return ::pressio::ops::dot(a, b);
}

auto dot_fc(py_f_arr a, py_c_arr b){
  return ::pressio::ops::dot(a, b);
}

auto dot_cf(py_c_arr a, py_f_arr b){
  return ::pressio::ops::dot(a, b);
}


// void myaddtodiagonal_f(py_f_arr a){
//   ::pressio::ops::add_to_diagonal(a, 5.5);
// }

// void myaddtodiagonal_c(py_c_arr a){
//   ::pressio::ops::add_to_diagonal(a, 5.5);
// }

// auto ewmult(nv_t y0, nv_t x0, nv_t z0)
// {
//   tensor_rank1_t y(y0);
//   tensor_rank1_t x(x0);
//   tensor_rank1_t z(z0);

//   ::pressio::ops::elementwise_multiply(1,x,z,1,y);
//   return *y.data();
// }

// auto vecUpd(nv_t v0, nv_t v01, nv_t v02)
// {
//   tensor_rank1_t v(v0);
//   tensor_rank1_t v1(v01);
//   tensor_rank1_t v2(v02);
//   ::pressio::ops::update(v, 2., v1, 2., v2, 2.);
//   return *v.data();
// }

// bool prodR3AR2BR2C()
// {
//   using r3_t = pressio::containers::Tensor<3,pressio4py::py_f_arr>;
//   using r2_t = pressio::containers::Tensor<2,pressio4py::py_f_arr>;

//   r3_t A(8,3,2);
//   r2_t B(8,3);
//   r2_t C(8,2);
//   pressio::ops::product(pressio::transpose(),
// 			pressio::nontranspose(),
// 			1., A, B, 1., C);
//   return true;
// }

}//anonym namespace

PYBIND11_MODULE(test_ops_module, m)
{
  m.def("extent", &myextent_f);
  m.def("extent", &myextent_c);
  m.def("clone", &myclone_f);
  m.def("clone", &myclone_c);
  m.def("deep_copy", &deepcopy_f);
  m.def("deep_copy", &deepcopy_c);
  m.def("deep_copy", &deepcopy_fc);
  m.def("deep_copy", &deepcopy_cf);
  m.def("abs", &myabs_f);
  m.def("abs", &myabs_c);
  m.def("abs", &myabs_fc);
  m.def("abs", &myabs_cf);
  m.def("scale", &scale_f);
  m.def("scale", &scale_c);
  m.def("fill", &fill_f);
  m.def("fill", &fill_c);
  m.def("set_zero", &set_zero_f);
  m.def("set_zero", &set_zero_c);
  m.def("max", &max_f);
  m.def("max", &max_c);
  m.def("min", &min_f);
  m.def("min", &min_c);
  m.def("norm1", &norm1_f);
  m.def("norm1", &norm1_c);
  m.def("norm2", &norm2_f);
  m.def("norm2", &norm2_c);
  m.def("dot", &dot_f);
  m.def("dot", &dot_c);

  // m.def("add_to_diagonal", &myaddtodiagonal_f);
  // m.def("add_to_diagonal", &myaddtodiagonal_c);
  // m.def("ewmultTest", &ewmult);
  // m.def("vecUpdTest", &vecUpd);
  //m.def("prodR3AR2BR2C", &prodR3AR2BR2C);
}
