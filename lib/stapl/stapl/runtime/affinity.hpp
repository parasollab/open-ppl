/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_AFFINITY_HPP
#define STAPL_RUNTIME_AFFINITY_HPP

#include "config.hpp"
#include "type_traits/is_basic.hpp"
#include <iosfwd>
#include <limits>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Describes the affinity of a location.
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
struct affinity_tag
{
  typedef runtime::internal_affinity_tag tag_type;

  static const std::size_t num_bits;

  static constexpr affinity_tag make_tag(const tag_type t) noexcept
  { return { t }; }

  tag_type tag;
};


constexpr bool operator==(const affinity_tag x, const affinity_tag y) noexcept
{
  return (x.tag==y.tag);
}

constexpr bool operator!=(const affinity_tag x, const affinity_tag y) noexcept
{
  return !(x==y);
}

std::ostream& operator<<(std::ostream&, const affinity_tag);


//////////////////////////////////////////////////////////////////////
/// @brief Invalid affinity tag.
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
constexpr affinity_tag invalid_affinity_tag =
  { std::numeric_limits<affinity_tag::tag_type>::max() };

} // namespace stapl

STAPL_IS_BASIC_TYPE(stapl::affinity_tag)

#endif
