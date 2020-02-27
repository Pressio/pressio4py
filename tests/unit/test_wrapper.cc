
#include "pressio_containers.hpp"
#include "types.hpp"

namespace{

bool constructWrapper1(typename pressio4py::CommonTypes::py_f_arr vIn, uintptr_t addr)
{
	using in_vec_t = typename pressio4py::CommonTypes::py_f_arr;
	using vec_t    = ::pressio::containers::Vector<in_vec_t>; 

	vec_t a(vIn);
	uintptr_t addr2 = reinterpret_cast<uintptr_t>(a.data()->data());
	std::cout << addr2 << " " << addr << std::endl;

	// the addresses must differ because a copies vIn when going 
	// from python to c++
	return addr2 != addr;
}

bool constructWrapper2(typename pressio4py::CommonTypes::py_f_arr vIn, uintptr_t addr)
{
	using in_vec_t = typename pressio4py::CommonTypes::py_f_arr;
	using vec_t    = ::pressio::containers::Vector<in_vec_t>; 

	vec_t a(vIn, pressio::view());
	uintptr_t addr2 = reinterpret_cast<uintptr_t>(a.data()->data());
	std::cout << addr2 << " " << addr << std::endl;

	// the addresses must be same because we view
	return addr2 == addr;
}


void constructWrapper3(typename pressio4py::CommonTypes::py_f_arr vIn, 
					   pybind11::object callBack)
{
	using in_vec_t = typename pressio4py::CommonTypes::py_f_arr;
	using vec_t    = ::pressio::containers::Vector<in_vec_t>; 

	vec_t a(vIn);
	uintptr_t addr2 = reinterpret_cast<uintptr_t>(a.data()->data());
	callBack.attr("cb")(*a.data(), addr2);
}

}

PYBIND11_MODULE(test_wrapper_module, m) {
    m.def("constructWrapper1", &constructWrapper1);
    m.def("constructWrapper2", &constructWrapper2);
    m.def("constructWrapper3", &constructWrapper3);
}
