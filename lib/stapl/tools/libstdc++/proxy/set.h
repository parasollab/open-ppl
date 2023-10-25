/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PROXY_SET_H
#define STAPL_PROXY_SET_H

#include <set>

#include <stapl/views/proxy/proxy.hpp>
#include <stapl/views/iterator/member_iterator.h>
#include <stapl/views/proxy_macros.hpp>

namespace stapl {

STAPL_PROXY_HEADER_TEMPLATE(std::set, T)
{
  STAPL_PROXY_DEFINES(std::set<T>)
  STAPL_PROXY_REFLECT_TYPE(size_type)
  STAPL_PROXY_METHOD_RETURN(size, size_type)
  STAPL_PROXY_METHOD_RETURN(max_size, size_type)
  STAPL_PROXY_METHOD_RETURN(empty, bool)

  typedef typename target_t::const_iterator            const_iter_t;
  typedef typename target_t::const_reverse_iterator    const_r_iter_t;

  typedef member_iterator<const_iter_t, Accessor>      const_iterator;
  typedef member_iterator<const_r_iter_t, Accessor>    const_r_iterator;

  typedef T                                            value_type;

  const_iterator begin() const
  {
    return const_iterator(Accessor::const_invoke(&target_t::begin), *this);
  }

  const_iterator end() const
  {
    return const_iterator(Accessor::const_invoke(&target_t::end), *this);
  }

  const_r_iterator rbegin() const
  {
    return const_r_iterator(Accessor::const_invoke(&target_t::rbegin), *this);
  }

  const_r_iterator rend() const
  {
    return const_r_iterator(Accessor::const_invoke(&target_t::rend), *this);
  }

  const_iterator cbegin() const
  {
    return const_iterator(Accessor::const_invoke(&target_t::cbegin), *this);
  }

  const_iterator cend() const
  {
    return const_iterator(Accessor::const_invoke(&target_t::cend), *this);
  }

  const_r_iterator crbegin() const
  {
    return const_r_iterator(Accessor::const_invoke(&target_t::crbegin), *this);
  }

  const_r_iterator crend() const
  {
    return const_r_iterator(Accessor::const_invoke(&target_t::crend), *this);
  }

}; // struct proxy

} // namespace stapl

#endif
