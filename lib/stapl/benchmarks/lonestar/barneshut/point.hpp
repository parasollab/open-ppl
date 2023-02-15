/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARK_LONESTAR_BH_POINT
#define STAPL_BENCHMARK_LONESTAR_BH_POINT

#include <stapl/runtime/runtime.hpp>

//////////////////////////////////////////////////////////////////////
/// @brief A 3D point in space
//////////////////////////////////////////////////////////////////////
struct point
{
  double x, y, z;

  point()
   : x(0.), y(0.), z(0.)
  { }

  point(double xx, double yy, double zz)
   : x(xx), y(yy), z(zz)
  { }

  void plus(point const& other)
  {
    x += other.x;
    y += other.y;
    z += other.z;
  }

  bool operator==(point const& other)
  {
    return std::abs(x-other.x) < FLT_EPSILON &&
           std::abs(y-other.y) < FLT_EPSILON &&
           std::abs(z-other.z) < FLT_EPSILON;
  }

  void define_type(stapl::typer& t)
  {
    t.member(x);
    t.member(y);
    t.member(z);
  }

};

#endif
