/*
//@HEADER
// ************************************************************************
//
// nonlinear_solvers.hpp
//                     		  Pressio
//                         Copyright 2019
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
    .value("standard",
	   pressio::solvers::nonlinear::update::standard)
    .value("armijo",
	   pressio::solvers::nonlinear::update::armijo)
    .value("LMSchedule1",
	   pressio::solvers::nonlinear::update::LMSchedule1)
    .value("LMSchedule2",
	   pressio::solvers::nonlinear::update::LMSchedule2)
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
  solverObj.def("maxIterations",
		&nonlinear_solver_t::maxIterations);
  solverObj.def("setMaxIterations",
		&nonlinear_solver_t::setMaxIterations);

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
//------------------------------------------------

template <typename T, typename = void>
struct has_system_typedef : std::false_type{};

template <typename T>
struct has_system_typedef<
  T, pressio::mpl::enable_if_t< !std::is_void<typename T::system_t >::value >
  > : std::true_type{};

template <typename T, typename = void>
struct has_stepper_typedef : std::false_type{};

template <typename T>
struct has_stepper_typedef<
  T, pressio::mpl::enable_if_t< !std::is_void<typename T::stepper_t >::value >
  > : std::true_type{};
//-----------------------------------------------

template<typename, typename = void>
struct _have_rj_api;

template<class T>
struct _have_rj_api<
  T, pressio::mpl::enable_if_t< has_system_typedef<T>::value >
  >
{
  static constexpr auto value =
    pressio::solvers::constraints::system_residual_jacobian<typename T::system_t>::value;
};

template<class T>
struct _have_rj_api<
  T, pressio::mpl::enable_if_t< has_stepper_typedef<T>::value >
  >
{
  static constexpr auto value =
    pressio::solvers::constraints::system_residual_jacobian<typename T::stepper_t>::value;
};
//------------------------------------------------

template<class...>
struct _have_rj_api_var;

template<class head>
struct _have_rj_api_var<head>
{
  static constexpr auto value = _have_rj_api<head>::value;
};

template<class head, class... tail>
struct _have_rj_api_var<head, tail...>
{
  static constexpr auto value = _have_rj_api<head>::value
    and _have_rj_api_var<tail...>::value;
};
//------------------------------------------------

// helper tags
struct UnweightedWls{};
struct Unweighted{};
struct Weighted{};
struct Irwls{};
struct NewtonRaphson{};

template<class nonlinear_solver_t, class rom_problem_t, class tagT>
pressio::mpl::enable_if_t<std::is_same<tagT, UnweightedWls>::value, nonlinear_solver_t>
createSolver(rom_problem_t & romProb,
	     const typename rom_problem_t::wls_native_state_t & romState,
	     pybind11::object pyobj) //linear solver for norm eq is a native python class
{
  return nonlinear_solver_t(romProb, romState, pyobj);
}

template<class nonlinear_solver_t, class rom_problem_t, class tagT>
pressio::mpl::enable_if_t<std::is_same<tagT, Unweighted>::value, nonlinear_solver_t>
createSolver(rom_problem_t & romProb,
	     const typename rom_problem_t::lspg_native_state_t & romState,
	     pybind11::object pyobj) //linear or QR solver is a native python class
{
  return nonlinear_solver_t(romProb, romState, pyobj);
}


template<class nonlinear_solver_t, class rom_problem_t, class tagT>
pressio::mpl::enable_if_t<std::is_same<tagT, Weighted>::value, nonlinear_solver_t>
createSolver(rom_problem_t & romProb,
	     const typename rom_problem_t::lspg_native_state_t & romState,
	     pybind11::object pyobj, // linear solver for norm eq is a native python class
	     pybind11::object wO)    // weighting operator is a native python class
{
  return nonlinear_solver_t(romProb, romState, pyobj, wO);
}

template<class nonlinear_solver_t, class rom_problem_t, class tagT>
pressio::mpl::enable_if_t<std::is_same<tagT, Irwls>::value, nonlinear_solver_t>
createSolver(rom_problem_t & romProb,
	     const typename rom_problem_t::lspg_native_state_t & romState,
	     pybind11::object pyobj, // linear solver for norm eq is a native python class
	     typename rom_problem_t::traits::scalar_t pNorm) // value of p-norm
{
  return nonlinear_solver_t(romProb, romState, pyobj, pNorm);
}

template<class nonlinear_solver_t, class rom_problem_t, class tagT>
pressio::mpl::enable_if_t<std::is_same<tagT, NewtonRaphson>::value, nonlinear_solver_t>
createSolver(rom_problem_t & romProb,
	     const typename rom_problem_t::galerkin_native_state_t & romState,
	     pybind11::object pyobj) //linear is a native python class
{
  return nonlinear_solver_t(romProb, romState, pyobj);
}

template<class ...>
struct bindCreateSolverVariadic;

template<class problem_type>
struct bindCreateSolverVariadic<problem_type>
{
  template<class nonlinear_solver_t, typename tag>
  static void bind(pybind11::module & m, const std::string & createFuncName)
  {
    m.def(createFuncName.c_str(),
	  &pressio4py::solvers::createSolver<nonlinear_solver_t, problem_type, tag>);
  }
};

template<class head, class ... tail>
struct bindCreateSolverVariadic<head, tail...>
{
  template<class nonlinear_solver_t, typename tag>
  static void bind(pybind11::module & m, const std::string & name)
  {
    bindCreateSolverVariadic<head>::template bind<nonlinear_solver_t, tag>(m, name);
    bindCreateSolverVariadic<tail...>::template bind<nonlinear_solver_t, tag>(m, name);
  }
};

//------------------------------------------------
/*   GN or LM, R/J API with normal equations    */
//------------------------------------------------
template<bool do_gn, typename ...>
struct LeastSquaresNormalEqResJacApiBinder;

template<bool do_gn, typename linear_solver_wrapper_t, typename ...Problems>
struct LeastSquaresNormalEqResJacApiBinder<do_gn, linear_solver_wrapper_t, std::tuple<Problems...>>
{
  static_assert(_have_rj_api_var<Problems...>::value, "");

  // it does not matter here if we use the steady system or stepper_t
  // as template arg to compose the solver type in the code below as long as it
  // meets the res-jac api. But since we are here, this condition is met
  // because it is asserted above. so just pick the first problem type in the
  // pack, which should be a steady lspg problem, and so it has a system_t typedef
  // that we can use for compose solver below
  using head_problem_t = typename std::tuple_element<0, std::tuple<Problems...>>::type;
  using system_t = typename head_problem_t::system_t;

  // gauss-newton solver type
  using gn_type = pressio::solvers::nonlinear::impl::composeGaussNewton_t
    <system_t, linear_solver_wrapper_t>;
  // lm solver type
  using lm_type = pressio::solvers::nonlinear::impl::composeLevenbergMarquardt_t
    <system_t, linear_solver_wrapper_t>;

  // pick gn or lm conditioned on the bool argument
  using nonlinear_solver_t = typename std::conditional<do_gn, gn_type, lm_type>::type;

  static void bindClass(pybind11::module & m, const std::string & solverPythonName)
  {
    pybind11::class_<nonlinear_solver_t> nonLinSolver(m, solverPythonName.c_str());
    bindTolerancesMethods(nonLinSolver);
    bindStoppingCriteria(nonLinSolver);
    bindCommonSolverMethods(nonLinSolver);

    // Note we don't bind the constructor because from Python we use the create
    // function (see below) to instantiate a solver object, we never
    // use the class name directly. This is useful because it allows us
    // to overcome the problem of needing unique class names in python
  }

  static void bindCreate(pybind11::module & m)
  {
    const std::string name = do_gn ? "createGaussNewton" : "createLevenbergMarquardt";
    bindCreateSolverVariadic<Problems...>::template bind<
      nonlinear_solver_t, Unweighted>(m, name);
  }
};

//------------------------------------------------
/*	GN, R/H API solved with QR		*/
//------------------------------------------------
template<bool do_gn, class ...>
struct LeastSquaresQRBinder;

template<bool do_gn, class qr_solver_t, class ...Problems>
struct LeastSquaresQRBinder<
  do_gn, qr_solver_t, std::tuple<Problems...>
  >
{
  static_assert(do_gn, "QR-based solver only supported for GN");
  static_assert(_have_rj_api_var<Problems...>::value, "");

  // it does not matter here if we use the steady system or stepper_t
  // as template arg to compose the solver type in the code below as long as it
  // meets the res-jac api. But since we are here, this condition is met
  // because it is asserted above. so just pick the first problem type in the
  // pack, which should be a steady lspg problem, and so it has a system_t typedef
  // that we can use for compose solver below
  using head_problem_t = typename std::tuple_element<0, std::tuple<Problems...>>::type;
  using system_t = typename head_problem_t::system_t;

  using nonlinear_solver_t =
    pressio::solvers::nonlinear::impl::composeGaussNewtonQR_t<system_t, qr_solver_t>;

  static void bindClass(pybind11::module & m, const std::string & solverPythonName)
  {
    pybind11::class_<nonlinear_solver_t> nonLinSolver(m, solverPythonName.c_str());
    bindTolerancesMethods(nonLinSolver);
    bindStoppingCriteria(nonLinSolver);
    bindCommonSolverMethods(nonLinSolver);

    // Note we don't bind the constructor because from Python we use the create
    // function (see below) to instantiate a solver object, we never
    // use the class name directly. This is useful because it allows us
    // to overcome the problem of needing unique class names in python
  }

  static void bindCreate(pybind11::module & m)
  {
    const std::string name = do_gn ? "createGaussNewtonQR" : "createLevenbergMarquardtQR";
    bindCreateSolverVariadic<Problems...>::template bind<
      nonlinear_solver_t, Unweighted>(m, name);
  }
};

//----------------------------------------------------
/* weighted GN or LM, R/J API with normal equations */
//----------------------------------------------------
template<bool do_gn, class ...>
struct WeightedLeastSquaresNormalEqBinder;

template<
  bool do_gn,
  typename linear_solver_t,
  typename weigher_t,
  class ... Problems
  >
struct WeightedLeastSquaresNormalEqBinder<
  do_gn, linear_solver_t, weigher_t, std::tuple<Problems...>
  >
{
  static_assert(_have_rj_api_var<Problems...>::value, "");

  // it does not matter here if we use the steady system or stepper_t
  // as template arg to compose the solver type in the code below as long as it
  // meets the res-jac api. But since we are here, this condition is met
  // because it is asserted above. so just pick the first problem type in the
  // pack, which should be a steady lspg problem, and so it has a system_t typedef
  // that we can use for compose solver below
  using head_problem_t = typename std::tuple_element<0, std::tuple<Problems...>>::type;
  using system_t = typename head_problem_t::system_t;

  // gauss-newton solver type
  using gn_type =
    pressio::solvers::nonlinear::impl::composeGaussNewton_t<system_t, linear_solver_t, weigher_t>;
  // lm solver type
  using lm_type =
    pressio::solvers::nonlinear::impl::composeLevenbergMarquardt_t<system_t, linear_solver_t, weigher_t>;

  // pick the final nonlin solver type is based on the do_gn
  using nonlinear_solver_t = typename std::conditional<do_gn, gn_type, lm_type>::type;

  static void bindClass(pybind11::module & m, const std::string & solverPythonName)
  {
    pybind11::class_<nonlinear_solver_t> nonLinSolver(m, solverPythonName.c_str());
    bindTolerancesMethods(nonLinSolver);
    bindStoppingCriteria(nonLinSolver);
    bindCommonSolverMethods(nonLinSolver);

    // Note we don't bind the constructor because from Python we use the create
    // function (see below) to instantiate a solver object, we never
    // use the class name directly. This is useful because it allows us
    // to overcome the problem of needing unique class names in python
  }

  static void bindCreate(pybind11::module & m)
  {
    const std::string name =
      do_gn ? "createWeightedGaussNewton" : "createWeightedLevenbergMarquardt";

    bindCreateSolverVariadic<Problems...>::template bind<
      nonlinear_solver_t, Weighted>(m, name);
  }
};

//----------------------------------------
/*	IRWGN normal equations		*/
//----------------------------------------
template<class ...>
struct IrwLeastSquaresNormalEqBinder;

template<class linear_solver_t, class ...Problems>
struct IrwLeastSquaresNormalEqBinder<linear_solver_t, std::tuple<Problems...>>
{
  static_assert(_have_rj_api_var<Problems...>::value, "");

  // it does not matter here if we use the steady system or stepper_t
  // as template arg to compose the solver type in the code below as long as it
  // meets the res-jac api. But since we are here, this condition is met
  // because it is asserted above. so just pick the first problem type in the
  // pack, which should be a steady lspg problem, and so it has a system_t typedef
  // that we can use for compose solver below
  using head_problem_t = typename std::tuple_element<0, std::tuple<Problems...>>::type;
  using system_t = typename head_problem_t::system_t;

  using composer_t = pressio::solvers::nonlinear::impl::composeIrwGaussNewton<system_t, linear_solver_t>;
  using w_t = typename composer_t::weighting_t;
  using nonlinear_solver_t = typename composer_t::type;

  static void bindClass(pybind11::module & m, const std::string & solverPythonName)
  {
    pybind11::class_<nonlinear_solver_t> nonLinSolver(m, solverPythonName.c_str());
    bindTolerancesMethods(nonLinSolver);
    bindStoppingCriteria(nonLinSolver);
    bindCommonSolverMethods(nonLinSolver);

    // Note we don't bind the constructor because from Python we use the create
    // function (see below) to instantiate a solver object, we never
    // use the class name directly. This is useful because it allows us
    // to overcome the problem of needing unique class names in python
  }

  static void bindCreate(pybind11::module & m)
  {
    const std::string name = "createIrwGaussNewton";
    bindCreateSolverVariadic<Problems...>::template bind<
      nonlinear_solver_t, Irwls>(m, name);
  }
};

//---------------------------------------------------
/* newton-raphson (only used for Galerkin for now) */
//---------------------------------------------------
template<bool dummy, class ...>
struct NewtonRaphsonBinder {};

template<bool dummy, class linear_solver_t, class ...Problems>
struct NewtonRaphsonBinder<dummy, linear_solver_t, std::tuple<Problems...>>
{
  using head_problem_t = typename std::tuple_element<0, std::tuple<Problems...>>::type;
  using system_t = typename head_problem_t::stepper_t;

  using nonlinear_solver_t =
    pressio::solvers::nonlinear::impl::composeNewtonRaphson_t<system_t, linear_solver_t>;

  static void bindClass(pybind11::module & m, const std::string & solverPythonName)
  {
    pybind11::class_<nonlinear_solver_t> nonLinSolver(m, solverPythonName.c_str());
    bindTolerancesMethods(nonLinSolver);
    bindStoppingCriteria(nonLinSolver);
    bindCommonSolverMethods(nonLinSolver);


    // Note we don't bind the constructor because from Python we use the create
    // function (see below) to instantiate a solver object, we never
    // use the class name directly. This is useful because it allows us
    // to overcome the problem of needing unique class names in python
  }

  static void bindCreate(pybind11::module & m)
  {
    const std::string name = "createNewtonRaphson";
    bindCreateSolverVariadic<Problems...>::template bind<
      nonlinear_solver_t, NewtonRaphson>(m, name);
  }
};

//-------------------------------------------------
/* GN or LM with normal equations, hess-grad api */
//-------------------------------------------------
template<bool do_gn, typename ...>
struct LeastSquaresNormalEqHessGrapApiBinder;

template<bool do_gn, typename linear_solver_wrapper_t, typename ...Problems>
struct LeastSquaresNormalEqHessGrapApiBinder<
  do_gn, linear_solver_wrapper_t, std::tuple<Problems...>
  >
{
  using system_t = typename std::tuple_element<0, std::tuple<Problems...>>::type;

  // gauss-newton solver type
  using gn_type = pressio::solvers::nonlinear::impl::composeGaussNewton_t
    <system_t, linear_solver_wrapper_t>;

  // lm solver type
  using lm_type = pressio::solvers::nonlinear::impl::composeLevenbergMarquardt_t
    <system_t, linear_solver_wrapper_t>;

  // pick gn or lm conditioned on the bool argument
  using nonlinear_solver_t = typename std::conditional<do_gn, gn_type, lm_type>::type;

  static void bindClass(pybind11::module & m, const std::string & solverPythonName)
  {
    pybind11::class_<nonlinear_solver_t> nonLinSolver(m, solverPythonName.c_str());
    bindTolerancesMethods(nonLinSolver);
    bindStoppingCriteria(nonLinSolver);
    bindCommonSolverMethods(nonLinSolver);

    // Note we don't bind the constructor because from Python we use the create
    // function (see below) to instantiate a solver object, we never
    // use the class name directly. This is useful because it allows us
    // to overcome the problem of needing unique class names in python
  }

  static void bindCreate(pybind11::module & m)
  {
    const std::string name = "createGaussNewton";
    bindCreateSolverVariadic<Problems...>::template bind<
      nonlinear_solver_t, UnweightedWls>(m, name);
  }
};


// //------------------------------------------------
// // helper metafunction for dealing with types in a tuple
// //------------------------------------------------
// template<template<bool, typename...> class T, bool, typename...>
// struct instantiate_from_tuple_pack { };

// template<
//   template<bool, typename...> class T,
//   bool b, class T1, typename... Ts
//   >
// struct instantiate_from_tuple_pack<T, b, T1, std::tuple<Ts...>>
// {
//   using type = T<b, T1, Ts...>;
// };

// template<
//   template<bool, typename...> class T,
//   bool b, class T1, class T2, typename... Ts
//   >
// struct instantiate_from_tuple_pack<T, b, T1, T2, std::tuple<Ts...>>
// {
//   using type = T<b, T1, T2, Ts...>;
// };

}}//end namespace
#endif
