/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_SPANS_PER_LOCATION_HPP
#define STAPL_SKELETONS_SPANS_PER_LOCATION_HPP

#include <stapl/skeletons/spans/balanced.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/utility/tuple/tuple.hpp>

namespace stapl {
namespace skeletons {
namespace spans {

//////////////////////////////////////////////////////////////////////
/// @brief A balanced span (@see balanced) in which each location
/// will have exactly one element to spawn.
///
/// @ingroup skeletonsSpans
//////////////////////////////////////////////////////////////////////
class per_location
  : public balanced<1>
{
public:
  using size_type = balanced<>::size_type;
  using dimension_type = balanced<>::dimension_type;

  template <bool forced = false, typename Spawner, typename... Views>
  void set_size(Spawner const& spawner, Views const&... views)
  {
    if (forced)
      forced_set_size(spawner, views...);
    else
      balanced::set_size(spawner,
                         stapl::counting_view<int>(spawner.get_num_PEs()));
  }

  template <typename Spawner, typename... Views>
  void forced_set_size(Spawner const& spawner, Views const&... views)
  {
    using VDomains =
      stapl::tuple<skeletons::domain_type<typename Views::domain_type,
                                          has_finite_domain<Views>::value>...>;
    auto view_domains =
      stapl::get<skeletons::first_finite_domain_index<VDomains>::value>(
        make_tuple(skeletons::domain_type<typename Views::domain_type,
                                          has_finite_domain<Views>::value>(
          views.domain())...));

    if (view_domains.size() < spawner.get_num_PEs()) {
      balanced::set_size(spawner, views...);
    } else {
      balanced::set_size(spawner,
                         stapl::counting_view<int>(spawner.get_num_PEs()));
    }
  }
};

} // namespace spans
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_SPANS_PER_LOCATION_HPP
