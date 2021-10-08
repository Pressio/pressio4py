/*
//@HEADER
// ************************************************************************
//
// py_decoder.hpp
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

#ifndef PY_DECODER_HPP_
#define PY_DECODER_HPP_

#include "pressio/type_traits.hpp"
#include "pressio/expressions.hpp"
#include "pressio/ops.hpp"

namespace pressio4py{

template <
  class ScalarType,
  class FomStateType,
  class JacobianType,
  class WrongLayoutJacobianType
  >
struct PyDecoder
{
  enum mappingKind{ Null, Linear, Custom };
  using jacobian_type  = JacobianType;
  using fom_state_type = FomStateType;
  using wrong_layout_jacobian_type = WrongLayoutJacobianType;

private:
  jacobian_type jacobian_ = {};
  mappingKind kind_ = {};
  pybind11::object customMapper_ = {};

public:
  PyDecoder() = delete;
  PyDecoder(const PyDecoder &) = delete;
  PyDecoder & operator=(const PyDecoder &) = delete;
  PyDecoder(PyDecoder &&) = delete;
  PyDecoder & operator=(PyDecoder &&) = delete;
  ~PyDecoder() = default;

  /*
    to make sure things work correctly, the jacobian
    is not deep copied but viewed here. To make sure this is right, if the type wrapped by
    jacobian_type is with a certain layout, we need to ensure a consistent
    object is passed to the constructor so that we can reference it.
    So we have two constructors for linear case, one accepting a native array
    with layout consisten with the wrapped type in jacobian_type and one accepting
    a numpy array with the "wrong" layout. if users calls the wrong one, an error is thrown.
   */
  explicit PyDecoder(jacobian_type jacobianMatrixIn)
    : jacobian_(jacobianMatrixIn),
      kind_(mappingKind::Linear)
  {
    // PRESSIOLOG_DEBUG
    //   ("linear: cnstr: matrix addr = {}, pyaddr = {}, size = ({},{})",
    //    fmt::ptr(&jacobian_), this->jacobianAddress(),
    //    jacobian_.extent(0), jacobian_.extent(1));
  }

  explicit PyDecoder(wrong_layout_jacobian_type jacobianMatrixIn)
  {
    constexpr auto neededColumnMajor = pressio::is_fstyle_array_pybind<jacobian_type>::value;
    constexpr auto passedRowMajor = pressio::is_cstyle_array_pybind<wrong_layout_jacobian_type>::value;

    // if needed jacobian is col major and pass is row major
    if (neededColumnMajor and passedRowMajor){
      throw std::runtime_error
	("The linear decoder needs a column-major jacobian but you are passing a row-major one.");
    }
    // if needed jacobian is row major and pass is col major
    else if (!neededColumnMajor and !passedRowMajor){
      throw std::runtime_error
	("The linear decoder needs a row-major jacobian but you are passing a column-major one.");
    }
    else{
      throw std::runtime_error
	("This case should never happen! Something is wrong!");
    }
  }

  // here the string description is not necessarily needed but it is important
  // to keep because it enables the right overload. Otherwise the interpreter
  // would pick this overload even if passing a numpy array because
  // a numpy array is also a python object.
  PyDecoder(pybind11::object customMapper, std::string description)
    // note that we "view" the native object, we don't deep copy it.
    // if the mapping jacobian changes on the python side, it reflects here.
    // NOTE that for this to work, the layout of the jacobian object
    // returned by "jacobian" MUST be same as the layout of the jacobian_type
    // otherwise pybind11 does not throw but just makes a copy of the data
    : jacobian_(customMapper.attr("jacobian")(/*no args*/)),
      kind_(mappingKind::Custom),
      customMapper_(customMapper)
  {
    // PRESSIOLOG_DEBUG
    //   ("custom: cnstr: matrix addr = {}, pyaddr = {}, size = ({},{})",
    //    fmt::ptr(&jacobian_), this->jacobianAddress(),
    //    jacobian_.extent(0), jacobian_.extent(1));
  }

  const jacobian_type & jacobianCRef() const{
    return jacobian_;
  }

  template<typename RomStateType>
  void updateJacobian(const RomStateType & rom_state)
  {
    if (kind_ == mappingKind::Linear){
      // no op
    }
    else if(kind_ == mappingKind::Custom)
    {
      customMapper_.attr("updateJacobian")(rom_state);
    }
    else{
      throw std::runtime_error("Invalid mapping kind enum");
    }
  }

  uintptr_t jacobianAddress() const{
    return reinterpret_cast<uintptr_t>(jacobian_.data());
  }

  // specialize for when operand is regular array
  template <class operand_t>
  pressio::mpl::enable_if_t<pressio::is_array_pybind<operand_t>::value>
  applyMapping(const operand_t & operand, fom_state_type & result) const
  {

    constexpr auto zero = ::pressio::utils::Constants<ScalarType>::zero();
    constexpr auto one  = ::pressio::utils::Constants<ScalarType>::one();
    if (kind_ == mappingKind::Linear)
    {
      if (jacobian_.ndim() == 2 and operand.ndim() == 1){
	::pressio::ops::product(pressio::nontranspose(), one,
				jacobian_, operand, zero, result);
      }
      else if (jacobian_.ndim() == 2 and operand.ndim() == 2){
	::pressio::ops::product(pressio::nontranspose(),
				pressio::nontranspose(), one,
				jacobian_, operand, zero, result);
      }
    }

    else if(kind_ == mappingKind::Custom){
      customMapper_.attr("applyMapping")(operand, result);
    }

    // else{
    //   throw std::runtime_error("decoder op(): Invalid mapping kind enum");
    // }
  }

};

}
#endif





  // template <class operand_t, class fom_state_to_compute_t, class _jacobian_t = jacobian_t>
  // void applyMapping(const pressio::containers::expressions::SpanExpr<operand_t> & operand,
  // 		    fom_state_to_compute_t & result) const
  // {
  //   // constexpr auto zero = ::pressio::utils::constants<ScalarType>::zero();
  //   // constexpr auto one  = ::pressio::utils::constants<ScalarType>::one();
  //   // if (kind_ == mappingKind::Linear)
  //   // {
  //   //   ::pressio::ops::product(pressio::nontranspose(), one,
  //   // 			      jacobian_, operand, zero, result);
  //   // }
  //   // else if(kind_ == mappingKind::Custom){
  //   //   throw std::runtime_error("If operand is an expression, custom mapping is not valid");
  //   // }
  //   // else
  //   //   throw std::runtime_error("Invalid mapping kind enum");
  // }

  // /*
  //   specialize for:
  //   - operand is a pressio span expression
  //   - result is a rank-1 tensor wrapper
  //   - jacobian is rank-2
  // */
  // template <class operand_t, class fom_state_to_compute_t, class _jacobian_t = jacobian_t>
  // pressio::mpl::enable_if_t<
  //   pressio::containers::predicates::is_rank1_tensor_wrapper_pybind<fom_state_to_compute_t>::value and
  //   _jacobian_t::traits::rank == 2
  //   >
  // applyMapping(const pressio::containers::expressions::SpanExpr<operand_t> & operand,
  // 	       fom_state_to_compute_t & result) const
  // {
  //   constexpr auto zero = ::pressio::utils::constants<ScalarType>::zero();
  //   constexpr auto one  = ::pressio::utils::constants<ScalarType>::one();
  //   if (kind_ == mappingKind::Linear)
  //   {
  //     ::pressio::ops::product(pressio::nontranspose(), one,
  // 			      jacobian_, operand, zero, result);
  //   }
  //   else if(kind_ == mappingKind::Custom){
  //     throw std::runtime_error("If operand is an expression, custom mapping is not valid");
  //   }
  //   else
  //     throw std::runtime_error("Invalid mapping kind enum");
  // }

  // /*
  //   specialize for: operand, result, jacobian are tensor wrappers with rank>=2
  // */
  // template <class operand_t, class fom_state_to_compute_t, class _jacobian_t = jacobian_t>
  // pressio::mpl::enable_if_t<
  //   pressio::containers::predicates::is_tensor_wrapper_pybind<operand_t>::value and
  //   pressio::containers::predicates::is_tensor_wrapper_pybind<fom_state_to_compute_t>::value and
  //   operand_t::traits::rank >=2 and
  //   fom_state_to_compute_t::traits::rank >= 2 and
  //   _jacobian_t::traits::rank >= 2
  //   >
  // applyMapping(const operand_t & operand, fom_state_to_compute_t & result) const
  // {

  //   constexpr auto zero = ::pressio::utils::constants<ScalarType>::zero();
  //   constexpr auto one  = ::pressio::utils::constants<ScalarType>::one();
  //   if (kind_ == mappingKind::Linear)
  //   {
  //     ::pressio::ops::product(pressio::nontranspose(),
  // 			      pressio::nontranspose(),
  // 			      one, jacobian_, operand, zero, result);
  //   }
  //   else if(kind_ == mappingKind::Custom){
  //     customMapper_.attr("applyMapping")(*operand.data(), *result.data());
  //   }
  //   else
  //     throw std::runtime_error("Invalid mapping kind enum");
  // }

  // /*
  //   specialize for operand and result being two native numpy arrays
  // */
  // template <class operand_t, class fom_state_to_compute_t, class _jacobian_t = jacobian_t>
  // pressio::mpl::enable_if_t<
  //   pressio::containers::predicates::is_array_pybind<operand_t>::value and
  //   pressio::containers::predicates::is_array_pybind<fom_state_to_compute_t>::value
  //   >
  // applyMapping(const operand_t & operand, fom_state_to_compute_t & result) const
  // {
  //   if (operand.ndim() != result.ndim())
  //     throw std::runtime_error("Operand and result cannot have different dimensions");

  //   /* we know the state rank from the class template argument
  //      so make sure the dims of the operand and result are consisten */
  //   if (operand.ndim()!=state_rank or result.ndim()!=state_rank){
  //     throw std::runtime_error
  // 	("Operand and/or result dims not matching rank from class template parameter");
  //   }

  //   constexpr auto zero = ::pressio::utils::constants<ScalarType>::zero();
  //   constexpr auto one  = ::pressio::utils::constants<ScalarType>::one();
  //   if (kind_ == mappingKind::Linear)
  //   {
  //     using w1_t = pressio::containers::Tensor<state_rank, operand_t>;
  //     using w2_t = pressio::containers::Tensor<state_rank, fom_state_to_compute_t>;
  //     w1_t oW(operand, pressio::view());
  //     w2_t rW(result,  pressio::view());
  //     this->applyMapping(oW, rW);
  //   }
  //   else if(kind_ == mappingKind::Custom){
  //     customMapper_.attr("applyMapping")(*operand.data(), *result.data());
  //   }
  //   else
  //     throw std::runtime_error("Invalid mapping kind enum");
  // }
