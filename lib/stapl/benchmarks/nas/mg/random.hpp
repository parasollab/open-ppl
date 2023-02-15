/*
// Copyright (c) 2000-2010, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_NAS_MG_RANDOM
#define STAPL_BENCHMARKS_NAS_MG_RANDOM

#include "../nas_rand.hpp"
#include "utils.hpp"

#include <algorithm>
#include <vector>

//////////////////////////////////////////////////////////////////////
/// @brief Helper struct representing a 3D point used to randomly
///        populate a 3D grid
//////////////////////////////////////////////////////////////////////
struct point
{
  int x, y, z;
  double value;

  point(int x_, int y_, int z_, double value_)
    : x(x_), y(y_), z(z_), value(value_)
  { }

  bool operator<(point const& p) const
  {
    return this->value < p.value;
  }

  bool operator>(point const& p) const
  {
    return this->value > p.value;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Densely fill a grid with random numbers.
///
/// @param v Grid to populate
//////////////////////////////////////////////////////////////////////
template<typename View>
void random_fill(View const& z)
{
  const double a  = std::pow(5.0, 13);
  double x0       = 314159265.0;

  randlc(x0, 1.0);

  for (size_t k = 0; k < stapl::get<2>(z.dimensions()); ++k)
    for (size_t j = 0; j < stapl::get<1>(z.dimensions()); ++j)
      for (size_t i = 0; i < stapl::get<0>(z.dimensions()); ++i)
        z(i,j,k) = randlc(x0, a);
}

//////////////////////////////////////////////////////////////////////
/// @brief Sparsely and randomly fill a grid with 1.0 and -1.0.
///
/// @param v Grid to populate
//////////////////////////////////////////////////////////////////////
template<typename View>
void zran3(View const& z)
{
  const int sample_size = 10;

  std::vector<point> small_pts(sample_size, point(-1, -1, -1, 1.0));
  std::vector<point> large_pts(sample_size, point(-1, -1, -1, 0.0));

  stapl::do_once([&]() {
    // (1) Populate z with random numbers in range 0..1
    random_fill(z);

    // (2) Find the coordinates of the sample_size largest as well as the
    // sample_size smallest random numbers in z.
    for (size_t i = 0; i < stapl::get<0>(z.dimensions()); ++i)
      for (size_t j = 0; j < stapl::get<1>(z.dimensions()); ++j)
      {
        for (size_t k = 0; k < stapl::get<2>(z.dimensions()); ++k)
        {
          if (z(i,j,k) < small_pts[9].value) {
            small_pts.back() = point(i, j, k, z(i,j,k));
            std::sort(small_pts.begin(), small_pts.end());
          }

          if (z(i,j,k) > large_pts[9].value) {
            large_pts.back() = point(i, j, k, z(i,j,k));
            std::sort(large_pts.begin(), large_pts.end(),
              std::greater<point>()
            );
          }
        }
     }
  });

  // (3) Reset to all zeros, setting the sample_size largest points to 1.0 and
  // the smallest to -1.0.
  zero3(z);

  stapl::do_once([&]() {
    for (point const& p : small_pts)
      z(p.x,p.y,p.z) = -1.0;

    for (point const& p : large_pts)
      z(p.x,p.y,p.z) = +1.0;
  });
}

#endif
