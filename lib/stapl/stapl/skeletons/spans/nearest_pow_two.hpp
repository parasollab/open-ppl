/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_SPANS_NEAREST_POW_TWO_HPP
#define STAPL_SKELETONS_SPANS_NEAREST_POW_TWO_HPP

#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/front.hpp>

namespace stapl {
namespace skeletons {
namespace spans {

/////////////////////////////////////////////////////////////////////////
/// @brief In some skeletons the number of elements to be spawned is less
/// than the elements in the input size. One good example is in the reduction
/// skeleton where the size of the leaves of the tree is less than or
/// equal to the number of elements in the input.
///
/// Typically, a skeleton with this span comes after a skeleton with
/// @c reduce_to_pow_two. The main difference between the two is that in
/// this span all the elements are spawned (no should_spawn is defined).
/// The reason behind this is that the size of the span is important to
/// the elementary skeleton using it, and hence needs to be stored.
///
/// @tparam OnSpan the span on which this tree is defined
///
/// @ingroup skeletonsSpans
/////////////////////////////////////////////////////////////////////////
template <typename OnSpan>
struct nearest_pow_two
  : public OnSpan
{
  using size_type      = typename OnSpan::size_type;
  using dimension_type = typename OnSpan::dimension_type;

  template <typename Spawner, typename... Views>
  void set_size(Spawner const& spawner, Views const&... views)
  {
    // we need to do this just to make sure we get the correct size for
    // the next step if something like spans::per_location is used.
    OnSpan::set_size(spawner, views...);
    std::size_t n = OnSpan::size();
    std::size_t nearest_pow_2 = 1;
    while (n != 1)
    {
      n >>= 1;
      nearest_pow_2 <<= 1;
    }
    //now we adjust the size
    //for now this is the only place that we are forcing the changes
    OnSpan::template set_size<true>(
      spawner, stapl::counting_view<int>(nearest_pow_2));
  }
};

} // namespace spans
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_SPANS_NEAREST_POW_TWO_HPP
