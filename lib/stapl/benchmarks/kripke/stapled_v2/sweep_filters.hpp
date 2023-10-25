/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_KRIPKE_SWEEP_FILTERS_HPP
#define STAPL_BENCHMARKS_KRIPKE_SWEEP_FILTERS_HPP

#include <vector>
#include "Grid.h"
#include "Kripke/Directions.h"
#include <stapl/skeletons/utility/position.hpp>
#include <stapl/skeletons/functional/wavefront.hpp>


// find the corner from which the sweep originates
inline
std::array<stapl::skeletons::position, 3>
find_corner(Directions const& sweep_direction)
{
  using stapl::skeletons::position;
  std::array<position, 3> corner;
  auto negative_face = position::first;
  auto positive_face = position::last;

  // If a sweep direction is positive along an axis, the sweep originates from
  // the negative face.
  corner[0] = sweep_direction.id > 0 ? negative_face : positive_face;
  corner[1] = sweep_direction.jd > 0 ? negative_face : positive_face;
  corner[2] = sweep_direction.kd > 0 ? negative_face : positive_face;

  return corner;
}

#endif // STAPL_BENCHMARKS_KRIPKE_SWEEP_FILTERS_HPP
