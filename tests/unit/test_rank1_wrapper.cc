
#include "pressio_containers.hpp"
#include "types.hpp"

using pressio4py::py_f_arr;
using pressio4py::py_c_arr;

// for rank-1 does not matter which one we use
using tr1_t = ::pressio::containers::Tensor<1, py_c_arr>;

namespace
{
void staticAsserts()
{
  using namespace pressio::containers;

  static_assert(!predicates::is_cstyle_array_pybind<py_f_arr>::value,"");
  static_assert(predicates::is_fstyle_array_pybind<py_f_arr>::value,"");
  static_assert(predicates::is_cstyle_array_pybind<py_c_arr>::value,"");
  static_assert(!predicates::is_fstyle_array_pybind<py_c_arr>::value,"");

  using tensor_f_t = Tensor<1, py_f_arr>;
  using tensor_c_t = Tensor<1, py_c_arr>;

  // must be a valid rank1 tensor
  static_assert(predicates::is_rank1_tensor_wrapper_pybind<tensor_f_t>::value,"");
  static_assert(predicates::is_rank1_tensor_wrapper_pybind<tensor_c_t>::value,"");

  // must NOT be valid rank2 or rank3 tensor
  static_assert(!predicates::is_cstyle_rank2_tensor_wrapper_pybind<tensor_c_t>::value,"");
  static_assert(!predicates::is_fstyle_rank2_tensor_wrapper_pybind<tensor_c_t>::value,"");
  static_assert(!predicates::is_cstyle_rank2_tensor_wrapper_pybind<tensor_f_t>::value,"");
  static_assert(!predicates::is_fstyle_rank2_tensor_wrapper_pybind<tensor_f_t>::value,"");

  static_assert(!predicates::is_cstyle_rank3_tensor_wrapper_pybind<tensor_c_t>::value,"");
  static_assert(!predicates::is_fstyle_rank3_tensor_wrapper_pybind<tensor_c_t>::value,"");
  static_assert(!predicates::is_cstyle_rank3_tensor_wrapper_pybind<tensor_f_t>::value,"");
  static_assert(!predicates::is_fstyle_rank3_tensor_wrapper_pybind<tensor_f_t>::value,"");
}

auto construct0(int extent)
{
  tr1_t a(extent);
  return *a.data();
}

bool construct1(py_f_arr vIn, uintptr_t addrInPython)
{
  // here vIn "views" the numpy array on python side
  // because pybind11::array has view semantics
  uintptr_t addrvIn = reinterpret_cast<uintptr_t>(vIn.data());
  if (addrvIn != addrInPython)
    throw std::runtime_error("addresses do not match");

  // if we use vIn to cosntruct a pressio tensor object,
  // the following constructor of the tensor container makes a copy
  tr1_t a(vIn);
  // so the address of the underlying object should be different
  uintptr_t addr2 = reinterpret_cast<uintptr_t>(a.data()->data());
  std::cout << addr2 << " " << addrInPython << std::endl;
  bool match = addr2 != addrInPython;

  return match;
}

bool construct2(py_f_arr vIn, uintptr_t addrInPython)
{
  // here vIn "views" the numpy array on python side
  // because pybind11::array has view semantics
  uintptr_t addrvIn = reinterpret_cast<uintptr_t>(vIn.data());
  if (addrvIn != addrInPython) return false;

  // use constructor with view semantics, so tensor views the same data
  tr1_t a(vIn, pressio::view());
  // so the address of the underlying object should be same
  uintptr_t addr2 = reinterpret_cast<uintptr_t>(a.data()->data());
  std::cout << addr2 << " " << addrInPython << std::endl;
  bool match = addr2 == addrInPython;
  return match;
}

bool copyConstruct()
{
  tr1_t a(3);
  a(0) = 1.1;
  a(1) = 2.2;
  a(2) = 3.3;

  tr1_t b(a);
  uintptr_t addr_a = reinterpret_cast<uintptr_t>(a.data()->data());
  uintptr_t addr_b = reinterpret_cast<uintptr_t>(b.data()->data());
  if (addr_a == addr_b) return false;

  if (b(0) != a(0)) return false;
  if (b(1) != a(1)) return false;
  if (b(2) != a(2)) return false;

  return true;
}

bool moveCstr(py_f_arr vIn, uintptr_t addr)
{
  //construct pressio wrapper
  tr1_t a(vIn);
  // the new data inside "a" must have differnet address
  //than the one from vIn
  uintptr_t addr2 = reinterpret_cast<uintptr_t>(a.data()->data());
  std::cout << addr2 << " " << addr << std::endl;
  if (addr2==addr) return false;

  tr1_t b(std::move(a));
  uintptr_t addr3 = reinterpret_cast<uintptr_t>(b.data()->data());
  std::cout << addr3 << " " << addr2 << std::endl;
  // addr3 must be same of a
  if (addr3 != addr2) return false;

  return true;
}

auto extent(py_f_arr vIn)
{
  tr1_t a(vIn);
  return a.extent(0);
}

auto subscript()
{
  tr1_t a(5);
  a(0) = 1.;
  // this should return by view so that we can check
  // on the python side if things are right
  return a(0)==1.;
}

void sliceCont(py_f_arr vIn)
{
  tr1_t a(vIn, pressio::view());
  if (a.extent(0) != 2) throw std::runtime_error("wrong extent");
  a(0) = 1.1;
  a(1) = 2.2;
}

void sliceNonCont(py_f_arr vIn, uintptr_t addrSlice)
{
  // since from python we pass a non-continguous slice
  // vIn is a pybind::array that is a copy of that slice
  uintptr_t addr2 = reinterpret_cast<uintptr_t>(vIn.data());
  std::cout << addr2 << " " << addrSlice << std::endl;
  if (addr2==addrSlice) throw std::runtime_error("addresses should not match");

  // even if I view vIn, vIn is just a copy of the python slice
  tr1_t a(vIn, pressio::view());
  std::cout << vIn.shape(0) << std::endl;
  if (a.extent(0) != 3) throw std::runtime_error("wrong extent");
  a(0) = 1.1;
  a(1) = 2.2;
}
}//anonym namespace

PYBIND11_MODULE(test_rank1_wrapper_module, m)
{
  m.def("construct0", &construct0);
  m.def("construct1", &construct1);
  m.def("construct2", &construct2);
  m.def("copyConstruct", &copyConstruct);
  m.def("moveCstr", &moveCstr);
  m.def("extent", &extent);
  m.def("subscript", &subscript);
  m.def("sliceContiguous", &sliceCont);
  m.def("sliceNonContiguous", &sliceNonCont);
}
