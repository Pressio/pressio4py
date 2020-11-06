/*
//@HEADER
// ************************************************************************
//
// nonlinear_solvers.hpp
//                     		  Pressio
//                             Copyright 2019
//    National Technology & Engineering Solutions of Sandia, LLC (NTESS)
//
// Under the terms of Contract DE-NA0003525 with NTESS, the
// U.S. Government retains certain rights in this software.
//
// Pressio is licensed under BSD-3-Clause terms of use:
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived
// from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
// IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact Francesco Rizzi (fnrizzi@sandia.gov)
//
// ************************************************************************
//@HEADER
*/

#ifndef PRESSIO4PY_PYBINDINGS_NONLINEAR_SOLVERS_HPP_
#define PRESSIO4PY_PYBINDINGS_NONLINEAR_SOLVERS_HPP_

namespace pressio4py{ namespace solvers{

void bindUpdatingEnums(pybind11::module & m)
{
  pybind11::enum_<pressio::solvers::nonlinear::update>(m, "update")
    .value("standard", pressio::solvers::nonlinear::update::standard)
    .value("armijo",   pressio::solvers::nonlinear::update::armijo)
    .value("LMSchedule1", pressio::solvers::nonlinear::update::LMSchedule1)
    .value("LMSchedule2", pressio::solvers::nonlinear::update::LMSchedule2)
    .export_values();
}

void bindStoppingEnums(pybind11::module & m)
{
  pybind11::enum_<pressio::solvers::nonlinear::stop>(m, "stop")
    .value("whenCorrectionAbsoluteNormBelowTolerance",
	   pressio::solvers::nonlinear::stop::whenCorrectionAbsoluteNormBelowTolerance)
    .value("whenCorrectionRelativeNormBelowTolerance",
	   pressio::solvers::nonlinear::stop::whenCorrectionRelativeNormBelowTolerance)
    .value("whenResidualAbsoluteNormBelowTolerance",
	   pressio::solvers::nonlinear::stop::whenResidualAbsoluteNormBelowTolerance)
    .value("whenResidualRelativeNormBelowTolerance",
	   pressio::solvers::nonlinear::stop::whenResidualRelativeNormBelowTolerance)
    .value("whenGradientAbsoluteNormBelowTolerance",
	   pressio::solvers::nonlinear::stop::whenGradientAbsoluteNormBelowTolerance)
    .value("whenGradientRelativeNormBelowTolerance",
	   pressio::solvers::nonlinear::stop::whenGradientRelativeNormBelowTolerance)
    .value("afterMaxIters",
	   pressio::solvers::nonlinear::stop::afterMaxIters)
    .export_values();
}

template <typename nonlinear_solver_t>
void bindCommonSolverMethods(pybind11::class_<nonlinear_solver_t> & solverObj)
{
  // methods to set and query num of iterations
  solverObj.def("maxIterations",    &nonlinear_solver_t::maxIterations);
  solverObj.def("setMaxIterations", &nonlinear_solver_t::setMaxIterations);

  // updating criterion
  solverObj.def("setUpdatingCriterion",
		&nonlinear_solver_t::setUpdatingCriterion);
  solverObj.def("updatingCriterion",
		&nonlinear_solver_t::updatingCriterion);
}

template <typename nonlinear_solver_t>
void bindTolerancesMethods(pybind11::class_<nonlinear_solver_t> & solverObj)
{
  // tolerances
  solverObj.def("setTolerance",
		&nonlinear_solver_t::setTolerance);

  solverObj.def("setCorrectionAbsoluteTolerance",
		&nonlinear_solver_t::setCorrectionAbsoluteTolerance);
  solverObj.def("setCorrectionRelativeTolerance",
		&nonlinear_solver_t::setCorrectionRelativeTolerance);
  solverObj.def("setResidualAbsoluteTolerance",
		&nonlinear_solver_t::setResidualAbsoluteTolerance);
  solverObj.def("setResidualRelativeTolerance",
		&nonlinear_solver_t::setResidualRelativeTolerance);
  solverObj.def("setGradientAbsoluteTolerance",
		&nonlinear_solver_t::setGradientAbsoluteTolerance);
  solverObj.def("setGradientRelativeTolerance",
		&nonlinear_solver_t::setGradientRelativeTolerance);

  solverObj.def("correctionAbsoluteTolerance",
		&nonlinear_solver_t::correctionAbsoluteTolerance);
  solverObj.def("correctionRelativeTolerance",
		&nonlinear_solver_t::correctionRelativeTolerance);
  solverObj.def("residualAbsoluteTolerance",
		&nonlinear_solver_t::residualAbsoluteTolerance);
  solverObj.def("residualRelativeTolerance",
		&nonlinear_solver_t::residualRelativeTolerance);
  solverObj.def("gradientAbsoluteTolerance",
		&nonlinear_solver_t::gradientAbsoluteTolerance);
  solverObj.def("gradientRelativeTolerance",
		&nonlinear_solver_t::gradientRelativeTolerance);
}

template <typename nonlinear_solver_t>
void bindStoppingCriteria(pybind11::class_<nonlinear_solver_t> & solverObj)
{
  // stopping criterion
  solverObj.def("setStoppingCriterion",
		&nonlinear_solver_t::setStoppingCriterion);
  solverObj.def("stoppingCriterion",
		&nonlinear_solver_t::stoppingCriterion);
};


template<
  bool do_gn,
  typename steady_prob_t, /*typename steady_system_t,*/
  typename prob_de_t,
  typename prob_hr_t,
  typename linear_solver_t,
  typename rom_native_state_t,
  typename rom_state_t
  >
struct LeastSquaresNormalEqBinder
{
  using steady_system_t = typename steady_prob_t::system_t;
  using stepper_de_t    = typename prob_de_t::stepper_t;
  using stepper_hr_t    = typename prob_hr_t::stepper_t;
  static_assert
  (::pressio::solvers::concepts::system_residual_jacobian<steady_system_t>::value,
   "Currently only supporting bindings for residual_jacobian_api");
  static_assert
  (::pressio::solvers::concepts::system_residual_jacobian<stepper_de_t>::value,
   "Currently only supporting bindings for residual_jacobian_api");
  static_assert
  (::pressio::solvers::concepts::system_residual_jacobian<stepper_hr_t>::value,
   "Currently only supporting bindings for residual_jacobian_api");

  // it does not matter here if we use stepper_t or system_t as template
  // args to declare the solver type in the code below as long as they
  // both meet the res-jac api. If we get here, it means that condition is met
  // because it is asserted above.
  using system_t = steady_system_t;

  using gn_type = pressio::solvers::nonlinear::composeGaussNewton_t<
    system_t, linear_solver_t>;
  using lm_type = pressio::solvers::nonlinear::composeLevenbergMarquardt_t<
    system_t, linear_solver_t>;
  using nonlinear_solver_t = typename std::conditional<do_gn, gn_type, lm_type>::type;

  static void bind(pybind11::module & m, std::string solverPythonName)
  {
    pybind11::class_<nonlinear_solver_t> nonLinSolver(m, solverPythonName.c_str());

    bindTolerancesMethods(nonLinSolver);
    bindStoppingCriteria(nonLinSolver);
    bindCommonSolverMethods(nonLinSolver);

    // -------------------------------------------
    // *** bindings for constructor for steady ***
    nonLinSolver.def(pybind11::init<
		     steady_prob_t &,
		     const rom_native_state_t &,
		     pybind11::object>());

    // // for steady LSPG, we also need to register the solve method
    // // because the state is owned by Python which calls solve directly
    // // so the c++ side takes in a rom_native_state_t
    // nonLinSolver.def("solve",
    // 		     &nonlinear_solver_t::template solve<system_t, rom_native_state_t>);

    // -----------------------------------------------------
    // *** bindings for constructor for unsteady default ***
    nonLinSolver.def(pybind11::init<
		     prob_de_t &,
		     const rom_native_state_t &,
		     pybind11::object>());

    // -----------------------------------------------------
    // *** bindings for constructor for unsteady hyp-red ***
    nonLinSolver.def(pybind11::init<
		     prob_hr_t &,
		     const rom_native_state_t &,
		     pybind11::object>());
  }
};

}}//end namespace
#endif
