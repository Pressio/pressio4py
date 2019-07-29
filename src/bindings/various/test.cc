
#include <iostream>
#include <Eigen/Core>
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
// #include <pybind11/eigen.h>
#include <pybind11/numpy.h>


// pybind11::array_t<double> add_arrays(pybind11::array_t<double> & input1,
// 				     pybind11::array_t<double> & input2)
// {
//   auto buf1 = input1.request(), buf2 = input2.request();

//   if (buf1.size != buf2.size)
//     throw std::runtime_error("Input shapes must match");

//   /*  allocate the buffer */
//   pybind11::array_t<double> result = pybind11::array_t<double>(buf1.size);

//   auto buf3 = result.request();
//   std::cout << buf1.size << " " << buf2.size << std::endl;

//   double *ptr1 = input1.mutable_data();
//   double *ptr2 = input2.mutable_data();
//   double *ptr3 = result.mutable_data();
//   int rows = buf1.shape[0];
//   int cols = buf1.shape[1];

//   for (size_t irow = 0; irow < rows; irow++){
//     for (size_t icol = 0; icol < cols; icol++){
//       ptr3[irow*cols + icol] = ptr1[irow*cols+ icol] + ptr2[irow*cols+ icol];
//       if (irow == icol and irow == 2)
// 	ptr3[irow*cols + icol] = 1.2;
//     }
//   }
//   // reshape array to match input shape
//   result.resize({rows,cols});

//   return result;
// }


struct Foo{
  using py_arr = pybind11::array_t<double>;
  py_arr a_;
  py_arr b_;

  void print(const py_arr & v){
    std::cout << v.at(0,1) << std::endl;
  }

  Foo(const py_arr & a) : a_(a){
    using py_arr = pybind11::array_t<double>;
    py_arr a1;
    py_arr b(a1);
    // auto b = a;
    std::cout << b.data() << " " << a1.data() << std::endl;

    // std::cout << "LSPGProbGen" << std::endl;
    // std::cout << "a shape " << a.shape()[0] << " " << a.shape()[1] << std::endl;
    // std::cout << "a ndim  " << a.ndim() << std::endl;
    // std::cout << "a  " << a.data() << " "
	   //    << a.owndata() << std::endl;
    // this->print(a);
    // std::cout << "a_ shape " << a_.shape()[0] << " " << a_.shape()[1] << std::endl;
    // std::cout << "a_ ndim  " << a_.ndim() << std::endl;
    // std::cout << "a_ " << a_.data() << " "
    // 	      << a_.owndata() << std::endl;
    // this->print(a_);

    // using py_arr = pybind11::array_t<double>;
    // py_arr a1;
    // auto b = a1;
    // std::cout << b.data() << " " << a1.data() << std::endl;

    // auto sh = *a.shape();
    // std::cout << sh << std::endl;
    // py_arr c( const_cast<py_arr&>(a).request().shape,
    // 	      const_cast<py_arr&>(a).request().strides,
    // 	      a.data() );

    // std::cout << "c ndim " << a.ndim() << std::endl;
    // std::cout << "c shape " << c.shape()[0] << " " << c.shape()[1] << std::endl;
    // std::cout << "c_ " << c.data() << " "
    // 	      << c.owndata() << std::endl;
    // this->print(c);
  }
};

PYBIND11_MODULE(pyrom, m) {
  using py_arr = pybind11::array_t<double>;

  pybind11::class_<Foo>(m, "Foo")
    .def(pybind11::init<py_arr &>());
}
