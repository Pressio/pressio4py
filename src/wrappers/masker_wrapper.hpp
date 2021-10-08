/*
//@HEADER
// ************************************************************************
//
// masker_wrapper.hpp
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

#ifndef PRESSIO4PY_PYBINDINGS_ROM_MASKER_WRAPPER_HPP_
#define PRESSIO4PY_PYBINDINGS_ROM_MASKER_WRAPPER_HPP_

namespace pressio4py{

class MaskerWrapper
{
  pybind11::object pyObj_;

public:
  explicit MaskerWrapper(pybind11::object pyObj)
    : pyObj_(pyObj){}

  MaskerWrapper() = delete;
  MaskerWrapper(const MaskerWrapper &) = default;
  MaskerWrapper & operator=(const MaskerWrapper &) = default;
  ~MaskerWrapper() = default;

public:
  template<typename T>
  T createApplyMaskResult(const T & operand) const{
    return pyObj_.attr("createApplyMaskResult")(operand);
  }

  template<class T, class ScalarType>
  void operator()(const T & operand,
		  const ScalarType time,
		  T & result) const
  {
    pyObj_.attr("__call__")(operand, time, result);
  }

  template<class T>
  void operator()(const T & operand, T & result) const
  {
    pyObj_.attr("__call__")(operand, result);
  }
};

}
#endif
