/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_UTILITY_FUNCTIONAL_HPP
#define STAPL_RUNTIME_UTILITY_FUNCTIONAL_HPP

#include "make_vector.hpp"
#include <algorithm>
#include <utility>
#include <vector>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Identity mapping function.
///
/// @tparam Arg    Type mapped from.
/// @tparam Result Type mapped to.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename Arg, typename Result = Arg>
struct identity
{
  Result operator()(Arg t) const
  { return Result(std::move(t)); }

  friend constexpr bool operator==(identity const&, identity const&) noexcept
  { return true; }

  friend constexpr bool operator!=(identity const&, identity const&) noexcept
  { return false; }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref identity when @p Arg and @p Result are the
///        same type.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename T>
struct identity<T, T>
{
  constexpr T const& operator()(T const& t) const noexcept
  { return t; }

  T operator()(T&& t) const
  { return std::move(t); }

  friend constexpr bool operator==(identity const&, identity const&) noexcept
  { return true; }

  friend constexpr bool operator!=(identity const&, identity const&) noexcept
  { return false; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Block mapping function.
///
/// @tparam Arg    Type mapped from.
/// @tparam Result Type mapped to.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename Arg, typename Result = Arg>
struct blocked
{
  Result m_blk_size;

  explicit blocked(Result r)
  : m_blk_size(std::move(r))
  { }

  Result operator()(Arg t) const
  { return (Result(std::move(t))/m_blk_size); }

  friend constexpr bool operator==(blocked const& x, blocked const& y) noexcept
  { return (x.m_blk_size==y.m_blk_size); }

  friend constexpr bool operator!=(blocked const& x, blocked const& y) noexcept
  { return !(x==y); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Fixed mapping function.
///
/// @tparam Result Type mapped to.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename Result>
struct fixed
{
  Result m_r;

  explicit fixed(Result r)
  : m_r(std::move(r))
  { }

  template<typename T>
  Result operator()(T&&) const
  { return m_r; }

  friend constexpr bool operator==(fixed const& x, fixed const& y) noexcept
  { return (x.m_r==y.m_r); }

  friend constexpr bool operator!=(fixed const& x, fixed const& y) noexcept
  { return !(x==y); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Arbitrary mapping function.
///
/// A range of values for the mapping has to be provided.
///
/// @tparam Arg    Type mapped from.
/// @tparam Result Type mapped to.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename Arg, typename Result = Arg>
struct arbitrary
{
  std::vector<Result> m_v;

  template<typename T,
           typename... U,
           typename = typename std::enable_if<
                        (sizeof...(U)>0) ||
                        !std::is_same<
                          typename std::decay<T>::type, arbitrary
                        >::value
                      >::type>
  explicit arbitrary(T&& t, U&&... u)
  : m_v(make_vector<Result>(std::forward<T>(t), std::forward<U>(u)...))
  { }

  Result const& operator()(Arg const& t) const noexcept
  { return m_v[t]; }

  friend bool operator==(arbitrary const& x, arbitrary const& y) noexcept
  {
    if (x.m_v.size()!=y.m_v.size())
      return false;
    return std::equal(x.m_v.begin(), x.m_v.end(), y.m_v.begin());
  }

  friend bool operator!=(arbitrary const& x, arbitrary const& y) noexcept
  { return !(x==y); }
};

} // namespace runtime

} // namespace stapl

#endif
