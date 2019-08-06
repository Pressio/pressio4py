
#ifndef PRESSIO_PYBINDINGS_GALERKIN_HPP_
#define PRESSIO_PYBINDINGS_GALERKIN_HPP_

#include "UTILS_ALL"
#include "CONTAINERS_ALL"
#include "ODE_ALL"
#include "ROM_GALERKIN"

#include "types.hpp"
#include "linear_decoder.cc"

PYBIND11_MODULE(pressio4pyGalerkin, m) {
  using mytypes = MyTypes;

  using scalar_t	= typename mytypes::scalar_t;
  using py_arr		= typename mytypes::py_arr;
  using fom_t		= typename mytypes::fom_t;
  using ops_t		= typename mytypes::ops_t;
  using rom_state_t	= typename mytypes::rom_state_t;
  using step_t		= typename mytypes::step_t;

  using decoderbind_t = LinearDecoderBinder<mytypes>;
  using decoder_t = typename decoderbind_t::decoder_t;

  //--------------------------------------------------------
  // Euler Galerkin problem
  //--------------------------------------------------------
  {
    constexpr auto ode_case  = ::pressio::ode::ExplicitEnum::Euler;
    using galerkin_problem_t = ::pressio::rom::DefaultGalerkinExplicitTypeGenerator<
      ode_case, rom_state_t, fom_t, decoder_t>;

    using galerkin_problem_gen = pressio::rom::GalerkinProblemGenerator<galerkin_problem_t>;

    using galerkin_stepper_t  = typename galerkin_problem_gen::galerkin_stepper_t;
    using res_pol_t	    = typename galerkin_problem_gen::galerkin_residual_policy_t;

    // stepper
    pybind11::class_<galerkin_stepper_t>(m, "StepperEuler")
      .def(pybind11::init<const rom_state_t &, const fom_t &, const res_pol_t &>());

    pybind11::class_<galerkin_problem_gen>(m, "ProblemEuler")
      .def(pybind11::init<const fom_t &, const py_arr &, decoder_t &,
	   rom_state_t &, scalar_t, const ops_t &>())
      .def("getStepper", &galerkin_problem_gen::getStepperRef);

    // integrator
    m.def("integrateNStepsEuler",
	  &::pressio::ode::integrateNSteps<
	  galerkin_stepper_t, rom_state_t, scalar_t, step_t>,
	  "Integrate N Steps");
  }
  //--------------------------------------------------------

  //--------------------------------------------------------
  // RK4 Galerkin problem
  //--------------------------------------------------------
  {
    constexpr auto ode_case  = ::pressio::ode::ExplicitEnum::RungeKutta4;
    using galerkin_problem_t = ::pressio::rom::DefaultGalerkinExplicitTypeGenerator<
      ode_case, rom_state_t, fom_t, decoder_t>;

    using galerkin_problem_gen = pressio::rom::GalerkinProblemGenerator<galerkin_problem_t>;

    using galerkin_stepper_t  = typename galerkin_problem_gen::galerkin_stepper_t;
    using res_pol_t	    = typename galerkin_problem_gen::galerkin_residual_policy_t;

    // stepper
    pybind11::class_<galerkin_stepper_t>(m, "StepperRK4")
      .def(pybind11::init<const rom_state_t &, const fom_t &, const res_pol_t &>());

    pybind11::class_<galerkin_problem_gen>(m, "ProblemRK4")
      .def(pybind11::init<const fom_t &, const py_arr &, decoder_t &,
	   rom_state_t &, scalar_t, const ops_t &>())
      .def("getStepper", &galerkin_problem_gen::getStepperRef);

    // integrator
    m.def("integrateNStepsRk4",
	  &::pressio::ode::integrateNSteps<
	  galerkin_stepper_t, rom_state_t, scalar_t, step_t>,
	  "Integrate N Steps");
  }
  //--------------------------------------------------------
}

#endif










// PYBIND11_MODULE(pressio4py, m) {

//   // type aliases
//   using scalar_t	= double;
//   using py_arr		= pybind11::array_t<scalar_t, pybind11::array::c_style>;
//   using fom_t		= pybind11::object;
//   using ops_t		= pybind11::object;
//   using step_t		= int;
//   using lspg_state_t	= py_arr;

//   // linear decoder
//   using decoder_jac_t	= py_arr;
//   using decoder_t	= ::pressio::rom::PyLinearDecoder<decoder_jac_t, ops_t>;

//   pybind11::class_<decoder_t>(m, "LinearDecoder")
//       .def(pybind11::init< const decoder_jac_t &, pybind11::object &>());

//   // lspg problem
//   // TODO: need to figure out how to handle enums from python
//   constexpr auto ode_case  = ::pressio::ode::ImplicitEnum::Euler;

//   using lspg_problem_type = pressio::rom::DefaultLSPGTypeGenerator<
//     fom_t, ode_case, decoder_t, lspg_state_t, ops_t>;
//   using lspg_prob_gen	= pressio::rom::LSPGUnsteadyProblemGenerator<lspg_problem_type>;
//   using lspg_stepper_t	= typename lspg_prob_gen::lspg_stepper_t;
//   using res_pol_t	= typename lspg_prob_gen::lspg_residual_policy_t;
//   using jac_pol_t	= typename lspg_prob_gen::lspg_jacobian_policy_t;

//   // concrete LSPG stepper
//   pybind11::class_<lspg_stepper_t>(m, "LspgStepperEuler")
//     .def(pybind11::init<const py_arr &, const fom_t &,
// 			const res_pol_t &, const jac_pol_t &>());

//   pybind11::class_<lspg_prob_gen>(m, "LspgProblemEuler")
//     .def(pybind11::init<const fom_t &, const py_arr &, decoder_t &,
// 			lspg_state_t &, scalar_t, const ops_t &>())
//     .def("getStepper", &lspg_prob_gen::getStepperRef);

//   // linear solver type: use pybind::object because we use numpy solver
//   using linear_solver_t = pybind11::object;

//   // non-linear solver type
//   using hessian_t = py_arr; // hessian is a numpy array
//   using nonlin_solver_t = ::pressio::solvers::iterative::PyGaussNewton
//     <lspg_stepper_t, py_arr, py_arr, py_arr,
//      hessian_t, linear_solver_t, scalar_t>;

//   // base types
//   using nls_base_t = ::pressio::solvers::NonLinearSolverBase<nonlin_solver_t>;
//   using iter_base_t = ::pressio::solvers::IterativeBase<scalar_t>;

//   pybind11::class_<nls_base_t>(m, "NonLinSolverBase")
//     .def("solve", &nls_base_t::template solve<lspg_stepper_t, py_arr>);

//   pybind11::class_<iter_base_t>(m, "IterBase")
//     .def("getMaxIterations", &iter_base_t::getMaxIterations)
//     .def("setMaxIterations", &iter_base_t::setMaxIterations)
//     .def("getTolerance", &iter_base_t::getTolerance)
//     .def("setTolerance", &iter_base_t::setTolerance);

//   pybind11::class_<nonlin_solver_t, iter_base_t, nls_base_t>(m, "GaussNewton")
//     .def(pybind11::init<const lspg_stepper_t &, const py_arr &,
// 			linear_solver_t &, pybind11::object &>());

//   // integrator
//   m.def("integrateNSteps",
//   	&::pressio::ode::integrateNSteps<
//   	lspg_stepper_t, py_arr, scalar_t, step_t, nonlin_solver_t>,
//   	"Integrate N Steps");
// }








  // pybind11::enum_<::pressio::ode::ImplicitEnum>(m, "ImplicitEnum", pybind11::arithmetic())
  //   .value("Undefined", ::pressio::ode::ImplicitEnum::Undefined)
  //   .value("Euler",	::pressio::ode::ImplicitEnum::Euler)
  //   .value("BDF2",	::pressio::ode::ImplicitEnum::BDF2);








// template <
//   ::pressio::ode::ImplicitEnum odeName,
//   typename scalar_t,
//   typename state_t,
//   typename residual_t,
//   typename jacobian_t
//   >
// struct BindingImplicitOde{

//   static void add(pybind11::module & m){
//     using app_t = pybind11::object;
//     using step_t = int;

//     //----------------------------------------------------------------------
//     // Bindings for stepper

//     constexpr auto baseName = PyClassHelper<odeName>::base_;
//     constexpr auto derivName = PyClassHelper<odeName>::derived_;
//     constexpr auto nAuxStates = PyClassHelper<odeName>::numAuxStates_;

//     // concrete stepper
//     using stepper_t = ::pressio::ode::ImplicitStepper
//       <odeName, state_t, residual_t, jacobian_t, app_t, scalar_t>;

//     // base stepper
//     using base_t = ::pressio::ode::ImplicitStepperBase<stepper_t, nAuxStates>;

//     pybind11::class_<base_t>(m, baseName)
//       .def("residual",
//     (residual_t (base_t::*)(const state_t &)const) &base_t::residual)
//       .def("residual",
//     (void (base_t::*)(const state_t &, residual_t &)const) &base_t::residual)
//       .def("jacobian",
//     (jacobian_t (base_t::*)(const state_t &)const) &base_t::jacobian)
//       .def("jacobian",
//     (void (base_t::*)(const state_t &, jacobian_t &)const) &base_t::jacobian);

//     // concrete stepper
//     pybind11::class_<stepper_t, base_t>(m, derivName)
//       .def(pybind11::init< const state_t &, const app_t & >());

//     //----------------------------------------------------------------------
//     // Bindings for Solver

//     // linear solver type
//     using linear_solver_t = pybind11::object;

//     // non-linear solver type
//     using hessian_t = jacobian_t;
//     using nonlin_solver_t = ::pressio::solvers::iterative::PyGaussNewton
//       <stepper_t, state_t, residual_t, jacobian_t, hessian_t, linear_solver_t, scalar_t>;

//     // base types
//     using nls_base_t = ::pressio::solvers::NonLinearSolverBase<nonlin_solver_t>;
//     using iter_base_t = ::pressio::solvers::IterativeBase<scalar_t>;

//     pybind11::class_<nls_base_t>(m, "NonLinSolverBase")
//       .def("solve", &nls_base_t::template solve<stepper_t, state_t>);

//     pybind11::class_<iter_base_t>(m, "IterBase")
//       .def("getMaxIterations", &iter_base_t::getMaxIterations)
//       .def("setMaxIterations", &iter_base_t::setMaxIterations)
//       .def("getTolerance", &iter_base_t::getTolerance)
//       .def("setTolerance", &iter_base_t::setTolerance);

//     pybind11::class_<nonlin_solver_t, iter_base_t, nls_base_t>(m, "GaussNewton")
//       .def(pybind11::init<
//           const stepper_t &,
//           const state_t &,
//           linear_solver_t &,
//     pybind11::object &>());

//     m.def("integrateNSteps",
//      &::pressio::ode::integrateNSteps<
//      stepper_t, state_t, scalar_t, step_t, nonlin_solver_t>,
//      "Integrate N Steps");
//   }
// };




// struct App{
//   using st_t = Eigen::VectorXd;
//   pybind11::object obj_;

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
//   .def(pybind11::init<const pybind11::object &>() );


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
// void test(pybind11::object & f)
// {
//   Eigen::MatrixXd A(2,2);
//   A.setConstant(2.2);

//   const pybind11::object & f_r = f.attr("residual");
//   f_r(A, 0);
//   std::cout << "GG " << A(0,0) << std::endl;
//   //pybind11::print( f_r(0) );
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
