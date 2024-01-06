/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef MTL_DIM_CALC_H
#define MTL_DIM_CALC_H

#include "mtl/fast.h"

namespace mtl {

//: For use in deciding whether to do static algorithm
//!noindex:
template <class Vector>
class dim_n {
public:
#if USE_BLAIS
  enum { N = (0 < Vector::N && Vector::N <= 15 ? Vector::N : 0) };
  typedef fast::count<N> RET;
#else
  typedef fast::count<0> RET;
#endif
};

//: For use in deciding whether to do static algorithm
//!noindex:
template <class Matrix>
class dim_m {
public:
#if USE_BLAIS
  enum { M = (0 < Matrix::M && Matrix::M <= 15 ? Matrix::M : 0) };
  typedef fast::count<M> RET;
#else
  typedef fast::count<0> RET;
#endif
};

} /* namespace mtl */

#endif /* MTL_DIM_CALC_H */
