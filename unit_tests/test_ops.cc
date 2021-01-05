
#include "pressio_ops.hpp"
#include "types.hpp"

namespace
{

using nv_t  = pressio4py::py_f_arr;
using tensor_rank1_t = ::pressio::containers::Tensor<1, nv_t>;

auto deepCopy(nv_t vIn)
{
  tensor_rank1_t a(vIn);
  tensor_rank1_t b(a.extent(0));
  ::pressio::ops::deep_copy(b,a);
  return *b.data();
}

auto norm1(nv_t vIn)
{
  tensor_rank1_t a(vIn);
  auto n1 = ::pressio::ops::norm1(a);
  return n1;
}

auto norm2(nv_t vIn)
{
  tensor_rank1_t a(vIn);
  auto n1 = ::pressio::ops::norm2(a);
  return n1;
}

auto dot(nv_t a0, nv_t b0)
{
  tensor_rank1_t a(a0);
  tensor_rank1_t b(b0);
  return ::pressio::ops::dot(a,b);
}

auto fill(nv_t a0)
{
  tensor_rank1_t a(a0);
  ::pressio::ops::fill(a,5.5);
  return *a.data();
}

auto scale(nv_t a0)
{
  tensor_rank1_t a(a0);
  ::pressio::ops::scale(a,2.);
  return *a.data();
}

auto ewmult(nv_t y0, nv_t x0, nv_t z0)
{
  tensor_rank1_t y(y0);
  tensor_rank1_t x(x0);
  tensor_rank1_t z(z0);

  ::pressio::ops::elementwise_multiply(1,x,z,1,y);
  return *y.data();
}

auto vecUpd(nv_t v0, nv_t v01, nv_t v02)
{
  tensor_rank1_t v(v0);
  tensor_rank1_t v1(v01);
  tensor_rank1_t v2(v02);
  ::pressio::ops::update(v, 2., v1, 2., v2, 2.);
  return *v.data();
}

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

PYBIND11_MODULE(test_ops_module, m) {
  m.def("deepCopyTest", &deepCopy);
  m.def("norm1Test", &norm1);
  m.def("norm2Test", &norm2);
  m.def("dotTest", &dot);
  m.def("fillTest", &fill);
  m.def("scaleTest", &scale);
  m.def("ewmultTest", &ewmult);
  m.def("vecUpdTest", &vecUpd);
  //m.def("prodR3AR2BR2C", &prodR3AR2BR2C);
}
