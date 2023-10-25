/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_SPANS_ONLY_NEAREST_POWER_TWO_HPP
#define STAPL_SKELETONS_SPANS_ONLY_NEAREST_POWER_TWO_HPP

namespace stapl {
namespace skeletons {
namespace spans {

//////////////////////////////////////////////////////////////////////
/// @brief Spans only the nearest power two element to the domain size.
///        It's only use currently is the @c pre_broadcast skeleton
///
/// @see pre_broadcast
/// @ingroup skeletonsSpans
//////////////////////////////////////////////////////////////////////
class only_nearest_pow_two
  : public spans::balanced<1>
{
public:
  using size_type      = std::size_t;
  using dimension_type = typename balanced<>::dimension_type;

  template <typename Coord>
  bool should_spawn(Coord const& skeleton_size, Coord const& coord) const
  {
    std::size_t i = tuple_ops::front(coord);
    std::size_t n = tuple_ops::front(skeleton_size);
    std::size_t nearest_pow2 = 1;
    while (n != 1) {
      nearest_pow2 <<= 1;
      n >>= 1;
    }
    return i == nearest_pow2 - 1;
  }
};

} // namespace spans
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_SPANS_ONLY_NEAREST_POWER_TWO_hpp
