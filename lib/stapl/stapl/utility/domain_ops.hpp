/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_DOMAIN_OPS_HPP
#define STAPL_UTILITY_DOMAIN_OPS_HPP

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Apply a function to each GID in a domain.
///
/// @param dom The domain to iterate over.
/// @param f   Function to call, which must accept a single GID as its
///            argument.
//////////////////////////////////////////////////////////////////////
template<typename Domain, typename F>
void domain_map(Domain const& dom, F&& f)
{
  if (dom.empty())
    return;

  typename Domain::index_type e = dom.first();
  typename Domain::index_type const& last = dom.last();

  for (; e != last; e = dom.advance(e, 1))
    f(e);

  // handle last element
  f(last);
}

} // namespace stapl

#endif // STAPL_UTILITY_DOMAIN_OPS_HPP
