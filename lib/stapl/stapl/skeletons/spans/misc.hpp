/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_SPANS_MISC_HPP
#define STAPL_SKELETONS_SPANS_MISC_HPP

#include <stapl/views/counting_view.hpp>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/front.hpp>
#include <stapl/skeletons/spans/balanced.hpp>
#include <stapl/skeletons/spans/per_location.hpp>
#include <stapl/skeletons/spans/reduce_to_pow_two.hpp>
#include <stapl/skeletons/spans/nearest_pow_two.hpp>

namespace stapl {
namespace skeletons {
namespace spans {

//////////////////////////////////////////////////////////////////////
/// @brief A span that will just have one element on the location it
/// was created on.
///
/// @ingroup skeletonsSpans
//////////////////////////////////////////////////////////////////////
class only_once
  : public balanced<1>
{
public:
  using size_type      = std::size_t;
  using dimension_type = typename balanced<>::dimension_type;

  template <typename Spawner, typename... Views>
  void set_size(Spawner const& spawner, Views const&... views)
  {
    using dom_t = skeletons::domain_type<indexed_domain<std::size_t>>;
    balanced::set_size(
      spawner, stapl::counting_view<int>(1));
  }
};

} // namespace spans
} // namespace skeletons
} // namepsace stapl

#endif // STAPL_SKELETONS_SPANS_MISC_HPP
