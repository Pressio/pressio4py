
#include <iostream>
#include <Eigen/Core>
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "../../../CORE_ALL"
#include "../../../ODE_ALL"

namespace py = pybind11;

struct updateOps{
  using scalar_t = double;
  using v_t = py::array_t<scalar_t, pybind11::array::c_style>;
  using this_t = updateOps;

  static void do_update(v_t & v, const scalar_t a,
			const v_t & v1, const scalar_t b,
			const v_t & v2, const scalar_t c,
			const v_t & v3, const scalar_t d,
			const v_t & v4, const scalar_t e) {
    const size_t sz = v.size();
    for (size_t i=0; i<sz; ++i){
      v.mutable_at(i) = a*v.at(i) +
    	b*v1.at(i) + c*v2.at(i) +
    	d*v3.at(i) + e*v4.at(i);
    }
  }
  static void do_update(v_t & v,
			const v_t & v1, const scalar_t b,
			const v_t & v2, const scalar_t c,
			const v_t & v3, const scalar_t d,
			const v_t & v4, const scalar_t e){
    this_t::do_update(v, 0, v1, b, v2, c, v3, d, v4, e);
  }

  static void do_update(v_t & v, const scalar_t a,
  			const v_t & v1, const scalar_t b){
    const size_t sz = v.size();
    for (size_t i=0; i< sz; ++i)
      v.mutable_at(i) = a*v.at(i) + b*v1.at(i);
  }

  static void do_update(v_t & v, const v_t & v1, const scalar_t b){
    this_t::do_update(v, 0, v1, b);
  }

  static void do_update(v_t & v, const scalar_t a,
  			const v_t & v1, const scalar_t b,
  			const v_t & v2, const scalar_t c){
    const size_t sz = v.size();
    for (size_t i=0; i<sz; ++i)
      v.mutable_at(i) = a*v.at(i) + b*v1.at(i) + c*v2.at(i);
  }

  static void do_update(v_t & v,
  			const v_t & v1, const scalar_t b,
  			const v_t & v2, const scalar_t c){
    this_t::do_update(v, 0, v1, b, v2, c);
  }
};

struct myops{
  using update_op = updateOps;
};


template <::rompp::ode::ExplicitEnum odeName>
struct PyClassName;

template <>
struct PyClassName<::rompp::ode::ExplicitEnum::Undefined>{
  static constexpr char const * base_ = "Undefined";
  static constexpr char const * derived_ = "Undefined";
};

template <>
struct PyClassName<::rompp::ode::ExplicitEnum::Euler>{
  static constexpr char const * base_ = "EEBase";
  static constexpr char const * derived_ = "ExplicitEuler";
};

template <>
struct PyClassName<::rompp::ode::ExplicitEnum::RungeKutta4>{
  static constexpr char const * base_ = "RK4Base";
  static constexpr char const * derived_ = "RungeKutta4";
};



template <
  ::rompp::ode::ExplicitEnum odeName,
  typename state_t,
  typename scalar_t
  >
struct BindingExplicitOde{

  static void add(pybind11::module & m){
    using app_t = py::object;
    using step_t = int;
    constexpr auto baseName = PyClassName<odeName>::base_;
    constexpr auto derivName = PyClassName<odeName>::derived_;

    // concrete stepper
    using stepper_t = ::rompp::ode::ExplicitStepper
      <odeName, state_t, app_t, state_t, scalar_t /*, myops*/>;

    // base stepper
    using base_t = ::rompp::ode::ExplicitStepperBase<stepper_t>;

    pybind11::class_<base_t>(m, baseName)
      .def("__call__", &base_t::template operator()<scalar_t>);

    pybind11::class_<stepper_t, base_t>(m, derivName)
      .def(pybind11::init<const state_t &, const app_t &>());

    m.def("integrateNSteps",
  	  &::rompp::ode::integrateNSteps<
  	  stepper_t, state_t, scalar_t, step_t>,
  	  "Integrate N Steps");
  }
};


PYBIND11_MODULE(pyode_expl, m) {

  using scalar_t = double;
  using py_state = py::array_t<scalar_t, pybind11::array::c_style>;
  using app_t = py::object;

  constexpr auto ee = ::rompp::ode::ExplicitEnum::Euler;
  BindingExplicitOde<ee, py_state, scalar_t>::add(m);

  constexpr auto rk4 = ::rompp::ode::ExplicitEnum::RungeKutta4;
  BindingExplicitOde<rk4, py_state, scalar_t>::add(m);
}















// struct App{
//   using st_t = Eigen::VectorXd;
//   py::object obj_;

//   App(const pybind11::object & obj)
//     : obj_(obj){}

//   void residual(const st_t & y,
// 		st_t & R,
// 		double t) const{
//     std::cout << "Inside APP2 " << std::endl;

//     pybind11::object f_r = obj_.attr("residual");
//     auto R1 = pybind11::cast<st_t>( f_r(y, R, t) );
//     std::cout << y << " \n" << R1 << std::endl;
//   }
// };

// pybind11::class_<App>(m, "App")
//   .def(pybind11::init<const py::object &>() );


// class ImplA;

// struct EulerImpl{
//   std::string a_ = {};

//   EulerImpl() = delete;
//   ~EulerImpl() = default;

//   EulerImpl(std::string a) : a_{a}{}

//   void do_step(){
//     std::cout << " fromImpl " << a_ << std::endl;
//   }
// };

// template <typename T>
// struct traits;

// template <>
// struct traits<ImplA>{
//   using impl_t = EulerImpl;
// };



// template<typename Impl>
// class Interface{
// public:
//   Interface() = default;
//   ~Interface() = default;
//   void operator()() {
//     static_cast<Impl*>(this)->compute_impl();
//   }

// };


// class ImplA : public Interface<ImplA>{
//   using base_t = Interface<ImplA>;
//   friend base_t;

//   using impl_class_t	= typename traits<ImplA>::impl_t;
//   impl_class_t myImpl_;

// public:
//   ImplA() = delete;
//   ~ImplA() = default;
//   ImplA(std::string a) : myImpl_(a){}

// private:
// template<typename ... Args2>
//   void compute_impl(Args2 && ... args){
//     myImpl_.do_step( std::forward<Args2>(args)... );
//   }
// };

// PYBIND11_MODULE(main, m) {
//   using type = Interface<ImplA>;
//   pybind11::class_<type>(m, "Interface<ImplA>")
//     .def("__call__", &type::operator());

//   pybind11::class_<ImplA, type>(m, "ImplA")
//     .def(pybind11::init<std::string>());
// }










// template <typename T>
// void foo(T a){
//   std::cout << "foo_py " << a(0) << std::endl;
// }


// //void run_test(const std::function<int(int)>& f)
// void test(py::object & f)
// {
//   Eigen::MatrixXd A(2,2);
//   A.setConstant(2.2);

//   const py::object & f_r = f.attr("residual");
//   f_r(A, 0);
//   std::cout << "GG " << A(0,0) << std::endl;
//   //py::print( f_r(0) );
//   //std::cout << "result: " << r << std::endl;
// }

// PYBIND11_MODULE(main, m) {
//   m.def("test", &test, "adds two numbers");
//   m.def("foo", &foo<Eigen::MatrixXd>, "dkdk");
// }

// // // int add(int i, int j) {
// // //     return i + j;
// // // }

// // PYBIND11_MODULE(main, m) {
// //     m.doc() = "pybind11 example plugin";
// //     m.def("add", &add, "A function which adds two numbers");
// // }
