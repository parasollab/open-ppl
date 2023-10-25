/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MAP_FUNCTIONAL_HPP
#define STAPL_CONTAINERS_MAP_FUNCTIONAL_HPP

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Function object that assigns that value field of a pair
/// to a given value
/// @ingroup pmapManip
/// @tparam T Type of the value of the pair
//////////////////////////////////////////////////////////////////////
template<typename T>
struct set_second
{
  T m_val;

  set_second(const T& val)
    : m_val(val)
  { }

  template<typename P>
  void operator()(P& p) const
  {
    p.second = m_val;
  }

  void define_type(typer& t)
  {
    t.member(m_val);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Dereferences an iterator over a pair and returns the
///   first element of that pair.
/// @ingroup pmapManip
//////////////////////////////////////////////////////////////////////
template <typename Gid>
struct get_first
{
  typedef Gid    gid_type;
  template <typename IT>
  Gid operator()(IT const& it) const
  {
    return (*it).first;
  }
};

} // namespace detail

} // namespace stapl

#endif // STAPL_CONTAINERS_MAP_FUNCTIONAL_HPP
