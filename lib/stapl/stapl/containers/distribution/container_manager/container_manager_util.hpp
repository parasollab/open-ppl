/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MANAGER_UTIL_HPP
#define STAPL_CONTAINERS_MANAGER_UTIL_HPP

#include <stapl/utility/tuple.hpp>

namespace stapl {

namespace cm_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Work function implementing the traversal of a tuple to compare
/// each element against known min and max values.
///
/// @tparam N current element of the tuple being examined
/// @tparam Index type of the full tuple being compared.
///
/// The work function is used to determine the bounds of the domains for
/// multiarray base containers.  This is needed because we currently find
/// the GIDs in each domain in their linear form and delinearize them to
/// get the necessary GID.  The multidimensional domain constructor
/// requires we pass the extrema of the domain to its constructor.
//////////////////////////////////////////////////////////////////////
template<int N, typename Index>
struct bc_min_max
{
private:
  Index& m_min;
  Index& m_max;

public:
  bc_min_max(Index& min, Index& max)
    : m_min(min), m_max(max)
  { }

  void operator()(Index const& val)
  {
    get<N>(m_min) = get<N>(val) < get<N>(m_min) ? get<N>(val) : get<N>(m_min);
    get<N>(m_max) = get<N>(val) > get<N>(m_max) ? get<N>(val) : get<N>(m_max);
    bc_min_max<N-1, Index>(m_min, m_max)(val);
  }
};


template<typename Index>
struct bc_min_max<0, Index>
{
private:
  Index& m_min;
  Index& m_max;

public:
  bc_min_max(Index& min, Index& max)
    : m_min(min), m_max(max)
  { }

  void operator()(Index const& val)
  {
    get<0>(m_min) = get<0>(val) < get<0>(m_min) ? get<0>(val) : get<0>(m_min);
    get<0>(m_max) = get<0>(val) > get<0>(m_max) ? get<0>(val) : get<0>(m_max);
  }
};

} // namespace cm_impl

} // namespace stapl

#endif // ifndef STAPL_CONTAINERS_MANAGER_UTIL_HPP
