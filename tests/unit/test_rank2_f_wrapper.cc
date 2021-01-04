
#include "pressio_containers.hpp"
#include "types.hpp"

using pressio4py::py_f_arr;
using pressio4py::py_c_arr;

namespace
{
void staticAsserts()
{
  using namespace pressio::containers;

  static_assert(!predicates::is_cstyle_array_pybind<py_f_arr>::value,"");
  static_assert(predicates::is_fstyle_array_pybind<py_f_arr>::value,"");
  static_assert(predicates::is_cstyle_array_pybind<py_c_arr>::value,"");
  static_assert(!predicates::is_fstyle_array_pybind<py_c_arr>::value,"");

  using tensor_f_t = Tensor<2, py_f_arr>;

  // must be a valid rank2 tensor
  static_assert(predicates::is_fstyle_rank2_tensor_wrapper_pybind<tensor_f_t>::value,"");
  // check for wrong layout
  static_assert(!predicates::is_cstyle_rank2_tensor_wrapper_pybind<tensor_f_t>::value,"");

  // must NOT be valid rank1 or rank3 tensor
  static_assert(!predicates::is_rank1_tensor_wrapper_pybind<tensor_f_t>::value,"");
  static_assert(!predicates::is_cstyle_rank3_tensor_wrapper_pybind<tensor_f_t>::value,"");
  static_assert(!predicates::is_fstyle_rank3_tensor_wrapper_pybind<tensor_f_t>::value,"");
}

using tf_t = ::pressio::containers::Tensor<2, py_f_arr>;
auto construct0(int s1, int s2)
{
  tf_t a(s1, s2);
  return *a.data();
}

// bool constructWrongLayout(py_c_arr vIn, uintptr_t addrInPython)
// {
//   // here vIn "views" the numpy array on python side
//   // because pybind11::array has view semantics
//   uintptr_t addrvIn = reinterpret_cast<uintptr_t>(vIn.data());
//   if (addrvIn != addrInPython) return false;

//   // tf_t is layout left, so this constructor should throw
//   // if constructor works, it means it is not right
//   try{
//     tf_t a(vIn);
//   }
//   catch (const std::runtime_error& error)
//   {
//     const std::string msg = "You cannot construct a rank-2 tensor wrapper with fortran layout from a row-major pybind::array";
//     assert( error.what() == msg );
//     return true;
//   }
//   return false;
// }

bool construct1(py_f_arr vIn, uintptr_t addrInPython)
{
  // here vIn "views" the numpy array on python side
  // because pybind11::array has view semantics
  uintptr_t addrvIn = reinterpret_cast<uintptr_t>(vIn.data());
  if (addrvIn != addrInPython) return false;

  // if we use vIn to cosntruct a pressio tensor object,
  // the following constructor of the tensor container makes a copy
  tf_t a(vIn);
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
  tf_t a(vIn, pressio::view());
  // so the address of the underlying object should be same
  uintptr_t addr2 = reinterpret_cast<uintptr_t>(a.data()->data());
  std::cout << addr2 << " " << addrInPython << std::endl;
  bool match = addr2 == addrInPython;
  return match;
}

bool construct2(py_c_arr vIn, uintptr_t addrInPython)
{
  // here vIn "views" the row-major numpy array on python side
  uintptr_t addrvIn = reinterpret_cast<uintptr_t>(vIn.data());
  if (addrvIn != addrInPython) return false;

  // use constructor with view semantics, but since
  // the tensor is col-major, it will create a copy
  // so the addresses should be different
  tf_t a(vIn, pressio::view());
  uintptr_t addr2 = reinterpret_cast<uintptr_t>(a.data()->data());
  std::cout << addr2 << " " << addrInPython << std::endl;
  return addr2 != addrInPython;
}

bool copyConstruct()
{
  tf_t a(3,3);
  a(0,0) = 1.1;
  a(1,0) = 2.2;
  a(2,1) = 3.3;

  tf_t b(a);
  uintptr_t addr_a = reinterpret_cast<uintptr_t>(a.data()->data());
  uintptr_t addr_b = reinterpret_cast<uintptr_t>(b.data()->data());
  if (addr_a == addr_b) return false;

  if (b(0,0) != a(0,0)) return false;
  if (b(1,0) != a(1,0)) return false;
  if (b(2,1) != a(2,1)) return false;
  return true;
}

bool moveCstr(py_f_arr vIn, uintptr_t addr)
{
  //construct pressio wrapper
  tf_t a(vIn);
  // the new data inside "a" must have differnet address
  //than the one from vIn
  uintptr_t addr2 = reinterpret_cast<uintptr_t>(a.data()->data());
  std::cout << addr2 << " " << addr << std::endl;
  if (addr2==addr) return false;

  tf_t b(std::move(a));
  uintptr_t addr3 = reinterpret_cast<uintptr_t>(b.data()->data());
  std::cout << addr3 << " " << addr2 << std::endl;
  // addr3 must be same of a
  if (addr3 != addr2) return false;

  return true;
}

bool extent(py_f_arr vIn)
{
  tf_t a(vIn);
  return (a.extent(0)==5 && a.extent(1)==6);
}

auto subscript()
{
  tf_t a(5,5);
  a(0,1) = 1.;
  // this should return by view so that we can check
  // on the python side if things are right
  return a(0,1)==1.;
}

void sliceContWithCorrectLayout(py_f_arr vIn)
{
  tf_t a(vIn, pressio::view());
  if (a.extent(0) != 5) throw std::runtime_error("wrong extent");
  if (a.extent(1) != 2) throw std::runtime_error("wrong extent");

  for (int i=0; i<5; ++i){
    a(i,0) = 2.;
    a(i,1) = 3.;
  }
}

// void sliceNonCont(py_f_arr vIn, uintptr_t addrSlice)
// {
//   // since from python we pass a non-continguous slice
//   // vIn is a pybind::array that is a copy of that slice
//   uintptr_t addr2 = reinterpret_cast<uintptr_t>(vIn.data());
//   std::cout << addr2 << " " << addrSlice << std::endl;
//   if (addr2==addrSlice) throw std::runtime_error("addresses should not match");

//   // even if I view vIn, vIn is just a copy of the python slice
//   tf_t a(vIn, pressio::view());
//   std::cout << vIn.shape(0) << std::endl;
//   if (a.extent(0) != 3) throw std::runtime_error("wrong extent");
//   a(0) = 1.1;
//   a(1) = 2.2;
// }
}//anonym namespace

PYBIND11_MODULE(test_rank2_f_wrapper_module, m)
{
  m.def("construct0", &construct0);
  m.def("construct1", &construct1);

  m.def("construct2",
	pybind11::overload_cast<py_c_arr, uintptr_t>(&construct2));
  m.def("construct2",
	pybind11::overload_cast<py_f_arr, uintptr_t>(&construct2));

  m.def("copyConstruct", &copyConstruct);
  m.def("moveCstr", &moveCstr);
  m.def("extent", &extent);
  m.def("subscript", &subscript);
  m.def("sliceContiguousWithCorrectLayout", &sliceContWithCorrectLayout);
  // m.def("sliceNonContiguous", &sliceNonCont);
}
