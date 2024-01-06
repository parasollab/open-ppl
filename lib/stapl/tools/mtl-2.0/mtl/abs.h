/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef MTL_STD_ABS_H
#define MTL_STD_ABS_H

#include "mtl/mtl_config.h"

#ifndef HAVE_STD_ABS
namespace std {
  inline double abs(double a) {
    return a > 0 ? a : -a;
  }
  inline long double abs(long double a) {
    return a > 0 ? a : -a;
  }
  inline float abs(float a) {
    return a > 0 ? a : -a;
  }
  inline long abs(long a) {
    return a > 0 ? a : -a;
  }
#if !defined ( _MSVCPP7_ )
  inline int abs(int a) {
    return a > 0 ? a : -a;
  }
#endif
}
#endif

#ifdef __SUNPRO_CC //sigh
namespace std {
  inline int abs(int a) {
    return a > 0 ? a : -a;
  }
}
#endif

#endif
