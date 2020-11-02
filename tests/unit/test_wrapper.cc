
#include "pressio_containers.hpp"
#include "types.hpp"

using Tf = typename pressio4py::CommonTypes::py_f_arr;
using Tc = typename pressio4py::CommonTypes::py_c_arr;
using vec_t = ::pressio::containers::Vector<Tf>;

namespace
{
void staticAsserts()
{
  static_assert
    (!pressio::containers::predicates::is_cstyle_array_pybind<Tf>::value,"");
  static_assert
    (pressio::containers::predicates::is_fstyle_array_pybind<Tf>::value,"");

  static_assert
    (pressio::containers::predicates::is_cstyle_array_pybind<Tc>::value,"");
  static_assert
    (!pressio::containers::predicates::is_fstyle_array_pybind<Tc>::value,"");

  using vf_t = ::pressio::containers::Vector<Tf>;
  using mf_t = ::pressio::containers::DenseMatrix<Tf>;
  using vc_t = ::pressio::containers::Vector<Tc>;
  using mc_t = ::pressio::containers::DenseMatrix<Tc>;

  static_assert
    (pressio::containers::predicates::is_cstyle_dense_matrix_wrapper_pybind<mc_t>::value,"");
  static_assert
    (!pressio::containers::predicates::is_fstyle_dense_matrix_wrapper_pybind<mc_t>::value,"");
  static_assert
    (pressio::containers::predicates::is_fstyle_dense_matrix_wrapper_pybind<mf_t>::value,"");
  static_assert
    (!pressio::containers::predicates::is_fstyle_dense_matrix_wrapper_pybind<mc_t>::value,"");

  static_assert
    (pressio::containers::predicates::is_vector_wrapper_pybind<vf_t>::value,"");
  static_assert
    (pressio::containers::predicates::is_vector_wrapper_pybind<vc_t>::value,"");
  static_assert
    (!pressio::containers::predicates::is_vector_wrapper_pybind<mf_t>::value, "");
  static_assert
    (!pressio::containers::predicates::is_vector_wrapper_pybind<mc_t>::value, "");

  static_assert
    (pressio::containers::predicates::is_vector_wrapper_pybind<vf_t>::value,"");
  static_assert
    (pressio::containers::predicates::is_vector_wrapper_pybind<vc_t>::value,"");
  static_assert
    (!pressio::containers::predicates::is_dense_matrix_wrapper_pybind<vf_t>::value, "");
  static_assert
    (!pressio::containers::predicates::is_dense_matrix_wrapper_pybind<vc_t>::value, "");
}

bool constructWrapper1(Tf vIn, uintptr_t addr)
{
  vec_t a(vIn);
  uintptr_t addr2 = reinterpret_cast<uintptr_t>(a.data()->data());
  std::cout << addr2 << " " << addr << std::endl;

  // addresses must differ because "a" copies "vIn"
  // when we go from python to c++
  return addr2 != addr;
}

bool constructWrapper2(Tf vIn,
		       uintptr_t addr)
{
  vec_t a(vIn, pressio::view());
  uintptr_t addr2 = reinterpret_cast<uintptr_t>(a.data()->data());
  std::cout << addr2 << " " << addr << std::endl;

  // the addresses must be same because we use pressio::view
  return addr2 == addr;
}

void constructWrapper3(Tf vIn,
		       pybind11::object callBack)
{
  vec_t a(vIn);
  uintptr_t addr2 = reinterpret_cast<uintptr_t>(a.data()->data());
  callBack.attr("cb")(*a.data(), addr2);
}

auto extent(Tf vIn)
{
  vec_t a(vIn);
  return a.extent(0);
}

auto subscriptChange(Tf vIn)
{
  vec_t a(vIn);
  a(0) = 1.;
  // this should return by view so that we can check
  // on the python side if things are right
  return *a.data();
}

bool moveCstr(Tf vIn,
		  uintptr_t addr)
{
  //construct pressio wrapper
  vec_t a(vIn);
  // the new data isnide a must have a differnet address
  //than the one from vIn
  uintptr_t addr2 = reinterpret_cast<uintptr_t>(a.data()->data());
  std::cout << addr2 << " " << addr << std::endl;
  if (addr2==addr) return false;

  vec_t b(std::move(a));
  uintptr_t addr3 = reinterpret_cast<uintptr_t>(b.data()->data());
  std::cout << addr3 << " " << addr2 << std::endl;
  // addr3 must be same of a
  if (addr3 != addr2) return false;

  return true;
}
}//anonym namespace

PYBIND11_MODULE(test_wrapper_module, m)
{
  m.def("constructWrapper1", &constructWrapper1);
  m.def("constructWrapper2", &constructWrapper2);
  m.def("constructWrapper3", &constructWrapper3);
  m.def("extent", &extent);
  m.def("subscriptChange", &subscriptChange);
  m.def("moveCstr", &moveCstr);
}
