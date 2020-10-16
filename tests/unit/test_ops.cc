
#include "pressio_ops.hpp"
#include "types.hpp"

namespace
{

using nv_t = typename pressio4py::CommonTypes::py_f_arr;
using vec_t    = ::pressio::containers::Vector<nv_t>;

auto deepCopy(nv_t vIn)
{
  vec_t a(vIn);
  vec_t b(a.extent(0));
  ::pressio::ops::deep_copy(b,a);
  return *b.data();
}

auto norm1(nv_t vIn)
{
  vec_t a(vIn);
  auto n1 = ::pressio::ops::norm1(a);
  return n1;
}

auto norm2(nv_t vIn)
{
  vec_t a(vIn);
  auto n1 = ::pressio::ops::norm2(a);
  return n1;
}

auto dot(nv_t a0, nv_t b0)
{
  vec_t a(a0);
  vec_t b(b0);
  return ::pressio::ops::dot(a,b);
}

auto fill(nv_t a0)
{
  vec_t a(a0);
  ::pressio::ops::fill(a,5.5);
  return *a.data();
}

auto scale(nv_t a0)
{
  vec_t a(a0);
  ::pressio::ops::scale(a,2.);
  return *a.data();
}

auto ewmult(nv_t y0, nv_t x0, nv_t z0)
{
  vec_t y(y0);
  vec_t x(x0);
  vec_t z(z0);

  ::pressio::ops::elementwise_multiply(1,x,z,1,y);
  return *y.data();
}

auto vecUpd(nv_t v0, nv_t v01, nv_t v02)
{
  vec_t v(v0);
  vec_t v1(v01);
  vec_t v2(v02);

  ::pressio::ops::update(v, 2., v1, 2., v2, 2.);
  return *v.data();
}

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
}
