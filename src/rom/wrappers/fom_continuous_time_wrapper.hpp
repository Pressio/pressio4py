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

#ifndef PRESSIO4PY_PYBINDINGS_ROM_FOM_CONTINUOUS_TIME_WRAPPER_HPP_
#define PRESSIO4PY_PYBINDINGS_ROM_FOM_CONTINUOUS_TIME_WRAPPER_HPP_

namespace pressio4py{ namespace rom{

template<
  typename scalar_t,
  typename state_t,
  typename velocity_t
  >
class FomWrapperCTimeNoApplyJac
{
protected:
  pybind11::object pyObj_;

public:
  using scalar_type       = scalar_t;
  using state_type	  = state_t;
  using velocity_type	  = velocity_t;

public:
  explicit FomWrapperCTimeNoApplyJac(pybind11::object pyObj)
    : pyObj_(pyObj){}

  FomWrapperCTimeNoApplyJac() = delete;
  FomWrapperCTimeNoApplyJac(const FomWrapperCTimeNoApplyJac&) = default;
  FomWrapperCTimeNoApplyJac & operator=(const FomWrapperCTimeNoApplyJac &) = default;
  FomWrapperCTimeNoApplyJac(FomWrapperCTimeNoApplyJac &&) = default;
  FomWrapperCTimeNoApplyJac & operator=(FomWrapperCTimeNoApplyJac &&) = default;
  ~FomWrapperCTimeNoApplyJac() = default;

public:
  velocity_type createVelocity() const{
    return pyObj_.attr("createVelocity")();
  }

  void velocity(const state_type & state,
		const scalar_type time,
		velocity_type & velo) const
  {
    pyObj_.attr("velocity")(state, time, velo);
  }
};

template<
  typename scalar_t,
  typename state_t,
  typename velocity_t,
  typename dense_matrix_t
  >
class FomWrapperCTimeWithApplyJac
  : public FomWrapperCTimeNoApplyJac<scalar_t, state_t, velocity_t>
{
public:
  using base_t = FomWrapperCTimeNoApplyJac<scalar_t, state_t, velocity_t>;
  using typename base_t::scalar_type;
  using typename base_t::state_type;
  using typename base_t::velocity_type;
  using base_t::pyObj_;

public:
  explicit FomWrapperCTimeWithApplyJac(pybind11::object pyObj)
    : base_t(pyObj){}

  FomWrapperCTimeWithApplyJac() = delete;
  FomWrapperCTimeWithApplyJac(const FomWrapperCTimeWithApplyJac &) = default;
  FomWrapperCTimeWithApplyJac & operator=(const FomWrapperCTimeWithApplyJac &) = default;
  FomWrapperCTimeWithApplyJac(FomWrapperCTimeWithApplyJac &&) = default;
  FomWrapperCTimeWithApplyJac & operator=(FomWrapperCTimeWithApplyJac &&) = default;
  ~FomWrapperCTimeWithApplyJac() = default;

public:
  dense_matrix_t createApplyJacobianResult(const dense_matrix_t & B) const{
    return pyObj_.attr("createApplyJacobianResult")(B);
  }

  void applyJacobian(const state_type & state,
		     const dense_matrix_t & operand,
		     const scalar_type time,
		     dense_matrix_t & result) const
  {
    pyObj_.attr("applyJacobian")(state, operand, time, result);
  }
};

}}//namespace pressio4py::rom
#endif
