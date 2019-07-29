
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include "../../../CORE_ALL"
#include "../../../SOLVERS_NONLINEAR"

namespace py = pybind11;

PYBIND11_MODULE(pysolver, m) {

  using scalar_t = double;
  using py_state = py::array_t<scalar_t, pybind11::array::c_style>;
  using system_t = py::object;

  // because the numpy linear solver is passed by user
  using lin_solver_t = py::object;

  // type of concrete nonlinear solver
  using nonlin_solver_t = ::rompp::solvers::NewtonRaphson<scalar_t, lin_solver_t>;
  // base types
  using nonlin_base_t = ::rompp::solvers::NonLinearSolverBase<nonlin_solver_t>;
  using iter_base_t = ::rompp::solvers::IterativeBase<scalar_t>;

  pybind11::class_<nonlin_base_t>(m, "NonLinBaseNewtonRaph")
    .def("solve", &nonlin_base_t::solve<system_t, py_state>);

  pybind11::class_<iter_base_t>(m, "IterBaseNewtonRaph")
    .def("getMaxIterations", &iter_base_t::getMaxIterations)
    .def("setMaxIterations", &iter_base_t::setMaxIterations)
    .def("getTolerance", &iter_base_t::getTolerance)
    .def("setTolerance", &iter_base_t::setTolerance);

  pybind11::class_<nonlin_solver_t, nonlin_base_t, iter_base_t>(m, "NewtonRaphson")
    .def(pybind11::init<>())
    .def(pybind11::init<lin_solver_t &>());

  //-------------------------------------
}









// template <
//   typename sc_t,
//   typename st_t,
//   typename res_t,
//   typename jac_t
//   >
// struct AppWrap{

//   py::object pyApp_;

//   using scalar_type = sc_t;
//   using state_type = st_t;
//   using residual_type = res_t;
//   using jacobian_type = jac_t;

//   AppWrap() = delete;
//   AppWrap(py::object & pyApp) : pyApp_{pyApp}{}
//   ~AppWrap() = default;

//   void residual(const state_type& x, residual_type& res) const {
//     pyApp_.attr("residual2")(x, res);
//   }

//   residual_type residual(const state_type& x) const {
//     auto res = pyApp_.attr("residual1")(x);
//     return res.template cast<residual_type>();
//   }

//   void jacobian(const state_type& x, jacobian_type& jac) const {
//     pyApp_.attr("jacobian2")(x, jac);
//   }

//   jacobian_type jacobian(const state_type& x) const {
//     auto jac = pyApp_.attr("jacobian1")(x);
//     return jac.template cast<jacobian_type>();
//   }
// };
