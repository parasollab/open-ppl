/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_DONTCARE_FLOW_HPP
#define STAPL_SKELETONS_DONTCARE_FLOW_HPP

#include <stapl/skeletons/flows/producer_info.hpp>
#include <stapl/skeletons/utility/utility.hpp>

namespace stapl {
namespace skeletons {
namespace flows {

////////////////////////////////////////////////////////////////////////
/// @brief You can think of a @c dontcare_flow as a flow that will
/// discard any requests coming to it. If it is queried for
/// @c consumers_count, it will return 0 and it also does not accept
/// read requests.
///
/// @ingroup skeletonsFlows
////////////////////////////////////////////////////////////////////////
class dontcare_flow
{
public:
  template <typename F = skeletons::no_filter>
  struct producer_type
  {
    using type = indexed_producer<void, std::size_t>;
  };

  template <typename Index,
            typename F = skeletons::no_filter,
            typename Mapper = skeletons::no_mapper>
  typename producer_type<F>::type
  consume_from(Index const& /*coord*/,
               F const& f = F(),
               Mapper const& mapper = Mapper()) const
  {
    return typename producer_type<F>::type(-1);
  }

  template <typename Coord>
  std::size_t consumer_count(Coord const& /*producer_coord*/) const
  {
    return 0;
  }
};

} // namespace flows
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_DONTCARE_FLOW_HPP
