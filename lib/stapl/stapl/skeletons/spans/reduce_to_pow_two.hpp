/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_SPANS_REDUCE_TO_POW_TWO_HPP
#define STAPL_SKELETONS_SPANS_REDUCE_TO_POW_TWO_HPP

#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/front.hpp>

namespace stapl {
namespace skeletons {
namespace spans {

/////////////////////////////////////////////////////////////////////////
/// @brief In some skeletons it is required to spawn as many elements as
/// the closest smaller power of two. This span defines that behavior.
///
/// For example, in a reduction tree of 20 elements, one needs to use a
/// reduction tree that has 16 leaves. In order to do that the size of
/// the input should be changed to the closest smaller power of two.
///
/// @tparam OnSpan the span on which this tree is defined
///
/// @ingroup skeletonsSpans
/////////////////////////////////////////////////////////////////////////
template <typename OnSpan>
struct reduce_to_pow_two
  : public OnSpan
{
public:
  using size_type = typename OnSpan::size_type;
  using dimension_type = typename OnSpan::dimension_type;

private:
  size_type m_nearest_pow_2;

public:
  template <typename Spawner, typename... Views>
  void set_size(Spawner const& spawner, Views const&... views)
  {
    auto view_domains = stapl::make_tuple(views.domain()...);
    using VDomains =
      stapl::tuple<skeletons::domain_type<typename Views::domain_type,
                                          has_finite_domain<Views>::value>...>;

    std::size_t n =
      stapl::get<skeletons::first_finite_domain_index<VDomains>::type::value>(
        view_domains)
        .size();

    m_nearest_pow_2 = 1;

    while (n != 1) {
      n >>= 1;
      m_nearest_pow_2 <<= 1;
    }
    OnSpan::set_size(spawner, views...);
  }

  template <typename Coord>
  bool should_spawn(Coord const& /* skeleton_size */, Coord const& coord) const
  {
    return ((std::size_t)tuple_ops::front(coord) < m_nearest_pow_2);
  }
};

} // namespace spans
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_SPANS_REDUCE_TO_POW_TWO_HPP
