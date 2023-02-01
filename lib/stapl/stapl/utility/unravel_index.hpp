/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_UNRAVEL_INDEX_HPP
#define STAPL_UTILITY_UNRAVEL_INDEX_HPP

#include <algorithm>
#include <numeric>
#include <deque>
#include <vector>

//////////////////////////////////////////////////////////////////////
/// @brief Convert a linear index to a multidimensional index where the
///        dimensionality is not known statically. This is based on
///        the unravel_index function in the NumPy package.
///
/// @param index Linear index in the space
/// @param shape Sizes of each dimension
/// @return An n-dimensional index
//////////////////////////////////////////////////////////////////////
std::vector<std::size_t> unravel_index(std::size_t index,
                                       std::vector<std::size_t> const& shape)
{
  std::deque<std::size_t> sizes(shape.begin(), shape.end());

  // Drop first element and reverse
  sizes.pop_front();
  std::reverse(std::begin(sizes), std::end(sizes));

  // Prepend 1
  sizes.push_front(1);

  // Calculate cummulative product
  std::partial_sum(std::begin(sizes), std::end(sizes), std::begin(sizes),
    std::multiplies<std::size_t>());

  // Reverse
  std::reverse(std::begin(sizes), std::end(sizes));

  // For each dim, divide by plane size and mod by dim size
  std::vector<std::size_t> unravelled(sizes.begin(), sizes.end());
  std::transform(
    std::begin(unravelled), std::end(unravelled), std::begin(shape),
    std::begin(unravelled), [&](std::size_t size, std::size_t shp) {
      return (index/size) % shp;
  });

  return unravelled;
}

#endif
