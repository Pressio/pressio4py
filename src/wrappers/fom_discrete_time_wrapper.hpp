/*
//@HEADER
// ************************************************************************
//
// fom_continuous_time_wrapper.hpp
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

#ifndef PRESSIO4PY_PYBINDINGS_ROM_FOM_DISCRETE_TIME_WRAPPER_HPP_
#define PRESSIO4PY_PYBINDINGS_ROM_FOM_DISCRETE_TIME_WRAPPER_HPP_

namespace pressio4py{ namespace rom{

template<
  int numStatesNeeded,
  typename scalar_t,
  typename state_t,
  typename residual_t,
  typename dense_matrix_t
  >
class FomWrapperDiscreteTime
{
private:
  pybind11::object pyObj_;

public:
  using scalar_type = scalar_t;
  using state_type = state_t;
  using discrete_time_residual_type = residual_t;

public:
  explicit FomWrapperDiscreteTime(pybind11::object pyObj)
    : pyObj_(pyObj){}

  FomWrapperDiscreteTime() = delete;
  FomWrapperDiscreteTime(const FomWrapperDiscreteTime &) = default;
  FomWrapperDiscreteTime & operator=(const FomWrapperDiscreteTime &) = default;
  ~FomWrapperDiscreteTime() = default;

public:
  discrete_time_residual_type createDiscreteTimeResidual() const{
    return pyObj_.attr("createDiscreteTimeResidual")();
  }

  dense_matrix_t createApplyDiscreteTimeJacobianResult(const dense_matrix_t & B) const{
    return pyObj_.attr("createApplyDiscreteTimeJacobianResult")(B);
  }

  template <typename step_t, typename ...Args>
  void discreteTimeResidual(const step_t & step,
			    const scalar_type & time,
			    const scalar_type & dt,
			    discrete_time_residual_type & R,
			    Args && ...args) const
  {
    discreteTimeResidualImpl(step, time, dt, R,
     			     std::forward<Args>(args)...);
  }

  template <typename step_t, typename ...Args>
  void applyDiscreteTimeJacobian(const step_t & step,
				 const scalar_type & time,
				 const scalar_type & dt,
				 const dense_matrix_t & B,
				 dense_matrix_t & A,
				 Args && ...args) const
  {
    applyDiscreteTimeJacobianImpl(step, time, dt, B, A,
				  std::forward<Args>(args)...);
  }

private:
  template <typename step_t, int _numStatesNeeded = numStatesNeeded>
  ::pressio::mpl::enable_if_t<_numStatesNeeded>=2>
  discreteTimeResidualImpl(const step_t & step,
			   const scalar_type & time,
			   const scalar_type & dt,
			   discrete_time_residual_type & R,
			   const state_t & ynp1,
			   const state_t & yn) const
  {
    pyObj_.attr("discreteTimeResidual")(step, time, dt, R, ynp1, yn);
  }

  template <typename step_t, int _numStatesNeeded = numStatesNeeded>
  ::pressio::mpl::enable_if_t<_numStatesNeeded==3>
  discreteTimeResidualImpl(const step_t & step,
			   const scalar_type & time,
			   const scalar_type & dt,
			   discrete_time_residual_type & R,
			   const state_t & ynp1,
			   const state_t & yn,
			   const state_t & ynm1) const
  {
    pyObj_.attr("discreteTimeResidual")(step, time, dt, R, ynp1, yn, ynm1);
  }

  template <typename step_t, int _numStatesNeeded = numStatesNeeded>
  ::pressio::mpl::enable_if_t<_numStatesNeeded>=2>
  applyDiscreteTimeJacobianImpl(const step_t & step,
				const scalar_type & time,
				const scalar_type & dt,
				const dense_matrix_t & B,
				dense_matrix_t & A,
				const state_t & ynp1,
				const state_t & yn) const
  {
    pyObj_.attr("applyDiscreteTimeJacobian")(step, time, dt, B, A, ynp1, yn);
  }

  template <typename step_t, int _numStatesNeeded = numStatesNeeded>
  ::pressio::mpl::enable_if_t<_numStatesNeeded==3>
  applyDiscreteTimeJacobianImpl(const step_t & step,
				const scalar_type & time,
				const scalar_type & dt,
				const dense_matrix_t & B,
				dense_matrix_t & A,
				const state_t & ynp1,
				const state_t & yn,
				const state_t & ynm1) const
  {
    pyObj_.attr("applyDiscreteTimeJacobian")(step, time, dt, B, A, ynp1, yn, ynm1);
  }

};

}}//namespace pressio4py::rom
#endif
