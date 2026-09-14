/*
//@HEADER
// ************************************************************************
//
// nonlinear_solvers.hpp
//                         Pressio
//                         Copyright 2019
//    National Technology & Engineering Solutions of Sandia, LLC (NTESS)
//
// Pressio is licensed under BSD-3-Clause terms of use.
// ************************************************************************
//@HEADER
*/

#ifndef PRESSIO4PY_PYBINDINGS_NONLINEAR_SOLVERS_HPP_
#define PRESSIO4PY_PYBINDINGS_NONLINEAR_SOLVERS_HPP_

#include <memory>
#include <utility>

namespace pressio4py{ namespace solvers{

void bindUpdatingEnums(pybind11::module & m)
{
  pybind11::enum_<pressio::nlsol::Update>(m, "update")
    .value("Standard", pressio::nlsol::Update::Standard)
    .value("Armijo", pressio::nlsol::Update::Armijo)
    .value("LMSchedule1", pressio::nlsol::Update::LMSchedule1)
    .value("LMSchedule2", pressio::nlsol::Update::LMSchedule2)
    .export_values();
}

void bindStoppingEnums(pybind11::module & m)
{
  // Preserve the historical pressio4py enum names while mapping them to the
  // Pressio 0.17 nlsol enum values.
  pybind11::enum_<pressio::nlsol::Stop>(m, "stop")
    .value("WhenCorrectionAbsoluteNormBelowTolerance",
           pressio::nlsol::Stop::WhenAbsolutel2NormOfCorrectionBelowTolerance)
    .value("WhenCorrectionRelativeNormBelowTolerance",
           pressio::nlsol::Stop::WhenRelativel2NormOfCorrectionBelowTolerance)
    .value("WhenResidualAbsoluteNormBelowTolerance",
           pressio::nlsol::Stop::WhenAbsolutel2NormOfResidualBelowTolerance)
    .value("WhenResidualRelativeNormBelowTolerance",
           pressio::nlsol::Stop::WhenRelativel2NormOfResidualBelowTolerance)
    .value("WhenGradientAbsoluteNormBelowTolerance",
           pressio::nlsol::Stop::WhenAbsolutel2NormOfGradientBelowTolerance)
    .value("WhenGradientRelativeNormBelowTolerance",
           pressio::nlsol::Stop::WhenRelativel2NormOfGradientBelowTolerance)
    .value("AfterMaxIters", pressio::nlsol::Stop::AfterMaxIters)
    .export_values();
}

class LegacySolverControls
{
  int maxIters_ = 100;
  pressio::nlsol::Update update_ = pressio::nlsol::Update::Standard;
  pressio::nlsol::Stop stop_ =
    pressio::nlsol::Stop::WhenAbsolutel2NormOfCorrectionBelowTolerance;

  double correctionAbsTol_ = 1.e-6;
  double correctionRelTol_ = 1.e-6;
  double residualAbsTol_ = 1.e-6;
  double residualRelTol_ = 1.e-6;
  double gradientAbsTol_ = 1.e-6;
  double gradientRelTol_ = 1.e-6;

  double toleranceForCurrentStop() const
  {
    using Stop = pressio::nlsol::Stop;
    switch (stop_){
      case Stop::WhenAbsolutel2NormOfCorrectionBelowTolerance: return correctionAbsTol_;
      case Stop::WhenRelativel2NormOfCorrectionBelowTolerance: return correctionRelTol_;
      case Stop::WhenAbsolutel2NormOfResidualBelowTolerance: return residualAbsTol_;
      case Stop::WhenRelativel2NormOfResidualBelowTolerance: return residualRelTol_;
      case Stop::WhenAbsolutel2NormOfGradientBelowTolerance: return gradientAbsTol_;
      case Stop::WhenRelativel2NormOfGradientBelowTolerance: return gradientRelTol_;
      default: return correctionAbsTol_;
    }
  }

public:
  template<class NativeSolver>
  void initialize(NativeSolver & solver) const
  {
    solver.setMaxIterations(maxIters_);
    solver.setUpdateCriterion(update_);
    solver.setStopCriterion(stop_);
    solver.setStopTolerance(toleranceForCurrentStop());
  }

  int maxIterations() const { return maxIters_; }
  pressio::nlsol::Update updatingCriterion() const { return update_; }
  pressio::nlsol::Stop stoppingCriterion() const { return stop_; }

  template<class NativeSolver>
  void setMaxIterations(NativeSolver & solver, int value)
  {
    maxIters_ = value;
    solver.setMaxIterations(value);
  }

  template<class NativeSolver>
  void setUpdatingCriterion(NativeSolver & solver, pressio::nlsol::Update value)
  {
    update_ = value;
    solver.setUpdateCriterion(value);
  }

  template<class NativeSolver>
  void setStoppingCriterion(NativeSolver & solver, pressio::nlsol::Stop value)
  {
    stop_ = value;
    solver.setStopCriterion(value);
    solver.setStopTolerance(toleranceForCurrentStop());
  }

  template<class NativeSolver>
  void setTolerance(NativeSolver & solver, double value)
  {
    correctionAbsTol_ = correctionRelTol_ = value;
    residualAbsTol_ = residualRelTol_ = value;
    gradientAbsTol_ = gradientRelTol_ = value;
    solver.setStopTolerance(value);
  }

  template<class NativeSolver>
  void setCorrectionAbsoluteTolerance(NativeSolver & solver, double value)
  {
    correctionAbsTol_ = value;
    if (stop_ == pressio::nlsol::Stop::WhenAbsolutel2NormOfCorrectionBelowTolerance)
      solver.setStopTolerance(value);
  }

  template<class NativeSolver>
  void setCorrectionRelativeTolerance(NativeSolver & solver, double value)
  {
    correctionRelTol_ = value;
    if (stop_ == pressio::nlsol::Stop::WhenRelativel2NormOfCorrectionBelowTolerance)
      solver.setStopTolerance(value);
  }

  template<class NativeSolver>
  void setResidualAbsoluteTolerance(NativeSolver & solver, double value)
  {
    residualAbsTol_ = value;
    if (stop_ == pressio::nlsol::Stop::WhenAbsolutel2NormOfResidualBelowTolerance)
      solver.setStopTolerance(value);
  }

  template<class NativeSolver>
  void setResidualRelativeTolerance(NativeSolver & solver, double value)
  {
    residualRelTol_ = value;
    if (stop_ == pressio::nlsol::Stop::WhenRelativel2NormOfResidualBelowTolerance)
      solver.setStopTolerance(value);
  }

  template<class NativeSolver>
  void setGradientAbsoluteTolerance(NativeSolver & solver, double value)
  {
    gradientAbsTol_ = value;
    if (stop_ == pressio::nlsol::Stop::WhenAbsolutel2NormOfGradientBelowTolerance)
      solver.setStopTolerance(value);
  }

  template<class NativeSolver>
  void setGradientRelativeTolerance(NativeSolver & solver, double value)
  {
    gradientRelTol_ = value;
    if (stop_ == pressio::nlsol::Stop::WhenRelativel2NormOfGradientBelowTolerance)
      solver.setStopTolerance(value);
  }

  double correctionAbsoluteTolerance() const { return correctionAbsTol_; }
  double correctionRelativeTolerance() const { return correctionRelTol_; }
  double residualAbsoluteTolerance() const { return residualAbsTol_; }
  double residualRelativeTolerance() const { return residualRelTol_; }
  double gradientAbsoluteTolerance() const { return gradientAbsTol_; }
  double gradientRelativeTolerance() const { return gradientRelTol_; }
};

struct NewtonFactory
{
  template<class SystemType, class LinearSolverType>
  static auto create(const SystemType & system, LinearSolverType & solver)
  {
    return pressio::nlsol::create_newton_solver(system, solver);
  }
};

struct GaussNewtonFactory
{
  template<class SystemType, class LinearSolverType>
  static auto create(const SystemType & system, LinearSolverType & solver)
  {
    return pressio::nlsol::create_gauss_newton_solver(system, solver);
  }
};

struct GaussNewtonQrFactory
{
  template<class SystemType, class QrSolverType>
  static auto create(const SystemType & system, QrSolverType & solver)
  {
    return pressio::nlsol::experimental::create_gauss_newton_qr_solver(system, solver);
  }
};

struct LevenbergMarquardtFactory
{
  template<class SystemType, class LinearSolverType>
  static auto create(const SystemType & system, LinearSolverType & solver)
  {
    return pressio::nlsol::create_levenberg_marquardt_solver(system, solver);
  }
};


template<class SystemType, class InnerSolverType, class Factory>
class OwnedNonlinearSolver
{
public:
  using state_type = typename SystemType::state_type;
  using native_solver_type = decltype(
    Factory::create(std::declval<const SystemType &>(), std::declval<InnerSolverType &>()));

private:
  std::unique_ptr<SystemType> system_;
  std::unique_ptr<InnerSolverType> innerSolver_;
  native_solver_type solver_;
  LegacySolverControls controls_;

public:
  OwnedNonlinearSolver(pybind11::object pySystem,
                       const state_type & state,
                       pybind11::object pyInnerSolver)
    : system_(std::make_unique<SystemType>(pySystem, state)),
      innerSolver_(std::make_unique<InnerSolverType>(pyInnerSolver)),
      solver_(Factory::create(*system_, *innerSolver_))
  {
    controls_.initialize(solver_);
  }

  OwnedNonlinearSolver() = delete;
  OwnedNonlinearSolver(const OwnedNonlinearSolver &) = delete;
  OwnedNonlinearSolver & operator=(const OwnedNonlinearSolver &) = delete;
  OwnedNonlinearSolver(OwnedNonlinearSolver &&) = default;
  OwnedNonlinearSolver & operator=(OwnedNonlinearSolver &&) = default;

  void solve(pybind11::object pySystem, state_type & state)
  {
    SystemType currentSystem(pySystem, state);
    solver_.solve(currentSystem, state);
  }

  int maxIterations() const { return controls_.maxIterations(); }
  void setMaxIterations(int value){ controls_.setMaxIterations(solver_, value); }
  auto updatingCriterion() const { return controls_.updatingCriterion(); }
  void setUpdatingCriterion(pressio::nlsol::Update value){ controls_.setUpdatingCriterion(solver_, value); }
  auto stoppingCriterion() const { return controls_.stoppingCriterion(); }
  void setStoppingCriterion(pressio::nlsol::Stop value){ controls_.setStoppingCriterion(solver_, value); }

  void setTolerance(double value){ controls_.setTolerance(solver_, value); }
  void setCorrectionAbsoluteTolerance(double value){ controls_.setCorrectionAbsoluteTolerance(solver_, value); }
  void setCorrectionRelativeTolerance(double value){ controls_.setCorrectionRelativeTolerance(solver_, value); }
  void setResidualAbsoluteTolerance(double value){ controls_.setResidualAbsoluteTolerance(solver_, value); }
  void setResidualRelativeTolerance(double value){ controls_.setResidualRelativeTolerance(solver_, value); }
  void setGradientAbsoluteTolerance(double value){ controls_.setGradientAbsoluteTolerance(solver_, value); }
  void setGradientRelativeTolerance(double value){ controls_.setGradientRelativeTolerance(solver_, value); }

  double correctionAbsoluteTolerance() const { return controls_.correctionAbsoluteTolerance(); }
  double correctionRelativeTolerance() const { return controls_.correctionRelativeTolerance(); }
  double residualAbsoluteTolerance() const { return controls_.residualAbsoluteTolerance(); }
  double residualRelativeTolerance() const { return controls_.residualRelativeTolerance(); }
  double gradientAbsoluteTolerance() const { return controls_.gradientAbsoluteTolerance(); }
  double gradientRelativeTolerance() const { return controls_.gradientRelativeTolerance(); }
};


template<class SystemType, class LinearSolverType, class WeightingType>
class OwnedWeightedGaussNewtonSolver
{
public:
  using state_type = typename SystemType::state_type;
  using native_solver_type = decltype(
    pressio::nlsol::create_gauss_newton_solver(
      std::declval<const SystemType &>(),
      std::declval<LinearSolverType &>(),
      std::declval<const WeightingType &>()));

private:
  std::unique_ptr<SystemType> system_;
  std::unique_ptr<LinearSolverType> linearSolver_;
  std::unique_ptr<WeightingType> weighting_;
  native_solver_type solver_;
  LegacySolverControls controls_;

public:
  OwnedWeightedGaussNewtonSolver(pybind11::object pySystem,
                                 const state_type & state,
                                 pybind11::object pyLinearSolver,
                                 pybind11::object pyWeighting)
    : system_(std::make_unique<SystemType>(pySystem, state)),
      linearSolver_(std::make_unique<LinearSolverType>(pyLinearSolver)),
      weighting_(std::make_unique<WeightingType>(pyWeighting)),
      solver_(pressio::nlsol::create_gauss_newton_solver(
        *system_, *linearSolver_, *weighting_))
  {
    controls_.initialize(solver_);
  }

  OwnedWeightedGaussNewtonSolver() = delete;
  OwnedWeightedGaussNewtonSolver(const OwnedWeightedGaussNewtonSolver &) = delete;
  OwnedWeightedGaussNewtonSolver & operator=(const OwnedWeightedGaussNewtonSolver &) = delete;
  OwnedWeightedGaussNewtonSolver(OwnedWeightedGaussNewtonSolver &&) = default;
  OwnedWeightedGaussNewtonSolver & operator=(OwnedWeightedGaussNewtonSolver &&) = default;

  void solve(pybind11::object pySystem, state_type & state)
  {
    SystemType currentSystem(pySystem, state);
    solver_.solve(currentSystem, state);
  }

  int maxIterations() const { return controls_.maxIterations(); }
  void setMaxIterations(int value){ controls_.setMaxIterations(solver_, value); }
  auto updatingCriterion() const { return controls_.updatingCriterion(); }
  void setUpdatingCriterion(pressio::nlsol::Update value){ controls_.setUpdatingCriterion(solver_, value); }
  auto stoppingCriterion() const { return controls_.stoppingCriterion(); }
  void setStoppingCriterion(pressio::nlsol::Stop value){ controls_.setStoppingCriterion(solver_, value); }

  void setTolerance(double value){ controls_.setTolerance(solver_, value); }
  void setCorrectionAbsoluteTolerance(double value){ controls_.setCorrectionAbsoluteTolerance(solver_, value); }
  void setCorrectionRelativeTolerance(double value){ controls_.setCorrectionRelativeTolerance(solver_, value); }
  void setResidualAbsoluteTolerance(double value){ controls_.setResidualAbsoluteTolerance(solver_, value); }
  void setResidualRelativeTolerance(double value){ controls_.setResidualRelativeTolerance(solver_, value); }
  void setGradientAbsoluteTolerance(double value){ controls_.setGradientAbsoluteTolerance(solver_, value); }
  void setGradientRelativeTolerance(double value){ controls_.setGradientRelativeTolerance(solver_, value); }

  double correctionAbsoluteTolerance() const { return controls_.correctionAbsoluteTolerance(); }
  double correctionRelativeTolerance() const { return controls_.correctionRelativeTolerance(); }
  double residualAbsoluteTolerance() const { return controls_.residualAbsoluteTolerance(); }
  double residualRelativeTolerance() const { return controls_.residualRelativeTolerance(); }
  double gradientAbsoluteTolerance() const { return controls_.gradientAbsoluteTolerance(); }
  double gradientRelativeTolerance() const { return controls_.gradientRelativeTolerance(); }
};


template<class SolverType>
void bindLegacySolverMethods(pybind11::class_<SolverType> & solver)
{
  solver.def("maxIterations", &SolverType::maxIterations)
    .def("setMaxIterations", &SolverType::setMaxIterations)
    .def("setUpdatingCriterion", &SolverType::setUpdatingCriterion)
    .def("updatingCriterion", &SolverType::updatingCriterion)
    .def("setStoppingCriterion", &SolverType::setStoppingCriterion)
    .def("stoppingCriterion", &SolverType::stoppingCriterion)
    .def("setTolerance", &SolverType::setTolerance)
    .def("setCorrectionAbsoluteTolerance", &SolverType::setCorrectionAbsoluteTolerance)
    .def("setCorrectionRelativeTolerance", &SolverType::setCorrectionRelativeTolerance)
    .def("setResidualAbsoluteTolerance", &SolverType::setResidualAbsoluteTolerance)
    .def("setResidualRelativeTolerance", &SolverType::setResidualRelativeTolerance)
    .def("setGradientAbsoluteTolerance", &SolverType::setGradientAbsoluteTolerance)
    .def("setGradientRelativeTolerance", &SolverType::setGradientRelativeTolerance)
    .def("correctionAbsoluteTolerance", &SolverType::correctionAbsoluteTolerance)
    .def("correctionRelativeTolerance", &SolverType::correctionRelativeTolerance)
    .def("residualAbsoluteTolerance", &SolverType::residualAbsoluteTolerance)
    .def("residualRelativeTolerance", &SolverType::residualRelativeTolerance)
    .def("gradientAbsoluteTolerance", &SolverType::gradientAbsoluteTolerance)
    .def("gradientRelativeTolerance", &SolverType::gradientRelativeTolerance)
    .def("solve", &SolverType::solve);
}


template<class linear_solver_t, class ResJacSystemWrapper>
struct NewtonRaphsonBinder
{
  using nonlinear_solver_t = OwnedNonlinearSolver<
    ResJacSystemWrapper, linear_solver_t, NewtonFactory>;

  static void bindClassAndMethods(pybind11::module & m)
  {
    pybind11::class_<nonlinear_solver_t> solver(m, "NewtonRaphClass");
    m.def("create_newton_raphson",
      [](pybind11::object system, const pressio4py::py_f_arr & state, pybind11::object linearSolver){
        return nonlinear_solver_t(system, state, linearSolver);
      });
    bindLegacySolverMethods(solver);
  }
};


template<class linear_solver_t, class ResJacSystemWrapper>
struct GNNormalEqResJacApiBinder
{
  using nonlinear_solver_t = OwnedNonlinearSolver<
    ResJacSystemWrapper, linear_solver_t, GaussNewtonFactory>;

  static void bindClassAndMethods(pybind11::module & m)
  {
    pybind11::class_<nonlinear_solver_t> solver(m, "GaussNewton");
    m.def("create_gauss_newton",
      [](pybind11::object system, const pressio4py::py_f_arr & state, pybind11::object linearSolver){
        return nonlinear_solver_t(system, state, linearSolver);
      });
    bindLegacySolverMethods(solver);
  }
};


template<class linear_solver_t, class ResJacSystemWrapper, class WeighWrapper>
struct WeighGNNormalEqResJacApiBinder
{
  using nonlinear_solver_t = OwnedWeightedGaussNewtonSolver<
    ResJacSystemWrapper, linear_solver_t, WeighWrapper>;

  static void bindClassAndMethods(pybind11::module & m)
  {
    pybind11::class_<nonlinear_solver_t> solver(m, "WeightedGaussNewton");
    m.def("create_weighted_gauss_newton",
      [](pybind11::object system, const pressio4py::py_f_arr & state,
         pybind11::object linearSolver, pybind11::object weighting){
        return nonlinear_solver_t(system, state, linearSolver, weighting);
      });
    bindLegacySolverMethods(solver);
  }
};


template<class qr_solver_t, class ResJacSystemWrapper>
struct GNQRResJacApiBinder
{
  using nonlinear_solver_t = OwnedNonlinearSolver<
    ResJacSystemWrapper, qr_solver_t, GaussNewtonQrFactory>;

  static void bindClassAndMethods(pybind11::module & m)
  {
    pybind11::class_<nonlinear_solver_t> solver(m, "GaussNewtonQR");
    m.def("create_gauss_newton_qr",
      [](pybind11::object system, const pressio4py::py_f_arr & state, pybind11::object qrSolver){
        return nonlinear_solver_t(system, state, qrSolver);
      });
    bindLegacySolverMethods(solver);
  }
};


template<class linear_solver_t, class ResJacSystemWrapper>
struct LMNormalEqResJacApiBinder
{
  using nonlinear_solver_t = OwnedNonlinearSolver<
    ResJacSystemWrapper, linear_solver_t, LevenbergMarquardtFactory>;

  static void bindClassAndMethods(pybind11::module & m)
  {
    pybind11::class_<nonlinear_solver_t> solver(m, "LevenbergMarquardt");
    m.def("create_levenberg_marquardt",
      [](pybind11::object system, const pressio4py::py_f_arr & state, pybind11::object linearSolver){
        return nonlinear_solver_t(system, state, linearSolver);
      });
    bindLegacySolverMethods(solver);
  }
};


template<class linear_solver_t, class ResJacSystemWrapper, class WeighWrapper>
struct WeighLMNormalEqResJacApiBinder
{
  static void bindClassAndMethods(pybind11::module & m)
  {
    // pressio-rom 0.17 no longer exposes a weighted LM factory.  Keep the
    // Python symbol so existing code gets an explicit compatibility error
    // rather than an AttributeError or a silently different algorithm.
    m.def("create_weighted_levenberg_marquardt",
      [](pybind11::object, const pressio4py::py_f_arr &,
         pybind11::object, pybind11::object) -> pybind11::object {
        throw std::runtime_error(
          "create_weighted_levenberg_marquardt is unavailable with pressio-rom 0.17: "
          "the upstream weighted Levenberg-Marquardt factory was removed");
      });
  }
};

}}//end namespace pressio4py::solvers
#endif
