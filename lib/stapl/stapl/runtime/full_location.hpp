/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_FULL_LOCATION_HPP
#define STAPL_RUNTIME_FULL_LOCATION_HPP

#include "runtime_fwd.hpp"
#include <iosfwd>
#include <functional>
#include <stapl/utility/hash_fwd.hpp>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Describes a location as a tuple of a gang id and a location id.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
class full_location
{
private:
  gang_id     m_gid;
  location_id m_lid;

public:
  constexpr explicit
  full_location(const gang_id gid = invalid_gang_id,
                const location_id lid = invalid_location_id) noexcept
  : m_gid(gid),
    m_lid(lid)
  { }

  constexpr gang_id get_gang_id(void) const noexcept
  { return m_gid; }

  constexpr location_id get_location_id(void) const noexcept
  { return m_lid; }

  constexpr bool valid(void) const noexcept
  { return ((m_gid!=invalid_gang_id) && (m_lid!=invalid_location_id)); }

  std::size_t hash_code(void) const noexcept
  {
    std::size_t seed = 0;
    boost::hash_combine(seed, get_gang_id());
    boost::hash_combine(seed, get_location_id());
    return seed;
  }

  friend constexpr bool operator==(full_location const& x,
                                   full_location const& y) noexcept
  {
    return ((x.m_lid==y.m_lid) && (x.m_gid==y.m_gid));
  }

  friend constexpr bool operator!=(full_location const& x,
                                   full_location const& y) noexcept
  {
    return !(x==y);
  }

  friend constexpr bool operator<(full_location const& x,
                                  full_location const& y) noexcept
  {
    return ((x.m_gid<y.m_gid)
              ? true
              : ((x.m_gid>y.m_gid) ? false : (x.m_lid<y.m_lid)));
  }

  friend constexpr bool operator>(full_location const& x,
                                  full_location const& y) noexcept
  {
    return (y<x);
  }
};

inline std::ostream& operator<<(std::ostream& os, full_location const& l)
{
  return os << '(' << l.get_gang_id() << ':' << l.get_location_id() << ')';
}

//////////////////////////////////////////////////////////////////////
/// @brief Hash value creation function for @ref full_location.
///
/// Required for @c boost::hash.
///
/// @related full_location
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
inline std::size_t hash_value(full_location const& l) noexcept
{
  return l.hash_code();
}

} // namespace runtime

} // namespace stapl


namespace std {

//////////////////////////////////////////////////////////////////////
/// @brief Hash value creation functor for @ref stapl::runtime::full_location.
///
/// @related stapl::runtime::full_location
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
template<>
struct hash<stapl::runtime::full_location>
{
  using argument_type = stapl::runtime::full_location;
  using result_type   = std::size_t;

  result_type operator()(argument_type const& t) const noexcept
  { return t.hash_code(); }
};

} // namespace std

#endif
