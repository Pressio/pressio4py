
#include <iostream>
#include <Eigen/Core>
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

pybind11::array_t<double> add_arrays(pybind11::array_t<double> & input1,
				     pybind11::array_t<double> & input2)
{
  auto buf1 = input1.request(), buf2 = input2.request();

  if (buf1.size != buf2.size)
    throw std::runtime_error("Input shapes must match");

  /*  allocate the buffer */
  pybind11::array_t<double> result = pybind11::array_t<double>(buf1.size);

  auto buf3 = result.request();
  std::cout << buf1.size << " " << buf2.size << std::endl;

  double *ptr1 = input1.mutable_data();
  double *ptr2 = input2.mutable_data();
  double *ptr3 = result.mutable_data();
  int rows = buf1.shape[0];
  int cols = buf1.shape[1];

  for (size_t irow = 0; irow < rows; irow++){
    for (size_t icol = 0; icol < cols; icol++){
      ptr3[irow*cols + icol] = ptr1[irow*cols+ icol] + ptr2[irow*cols+ icol];
      if (irow == icol and irow == 2)
	ptr3[irow*cols + icol] = 1.2;
    }
  }
  // reshape array to match input shape
  result.resize({rows,cols});

  return result;
}


struct TestClass{
  pybind11::array_t<double> a_;
  pybind11::array_t<double> b_;
  pybind11::object m_;

  TestClass(pybind11::object & m) : m_(m){
    a_.resize({3});
    b_.resize({3});
  }
  void run(){
    m_.attr("run2")(a_, b_);
    std::cout << a_.at(0) << std::endl;
  }
};

PYBIND11_MODULE(example, m) {
  m.doc() = "Add two vectors using pybind11"; // optional module docstring
  m.def("add_arrays", &add_arrays, "Add two NumPy arrays");

  pybind11::class_<TestClass>(m, "TestClass")
    .def(pybind11::init<pybind11::object &>())
    .def("run", &TestClass::run);
}
