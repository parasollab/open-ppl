/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PROXY_ARRAY_H
#define STAPL_PROXY_ARRAY_H

#include <stapl/views/proxy/proxy.hpp>
#include <stapl/views/iterator/member_iterator.h>
#include <stapl/views/proxy_macros.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @todo Make reflected const_reference type different from reference
///   when const_member_accessor and associated helper macros exist.
//////////////////////////////////////////////////////////////////////
template <typename T, std::size_t N, typename Accessor>
class proxy<std::array<T, N>, Accessor>
  : public Accessor
{
  typedef std::array<T, N> base_target_t;

  STAPL_PROXY_DEFINES(base_target_t)

public:
  STAPL_PROXY_REFLECT_TYPE(size_type)
  STAPL_PROXY_METHOD_RETURN(size, size_type)
  STAPL_PROXY_METHOD_RETURN(max_size, size_type)
  STAPL_PROXY_METHOD_RETURN(empty, bool)

  STAPL_PROXY_REFERENCE_METHOD_0(front, T)
  STAPL_PROXY_REFERENCE_METHOD_0(back, T)
  STAPL_PROXY_REFERENCE_METHOD_1(inner, operator[], T, std::size_t)
  STAPL_PROXY_REFERENCE_METHOD_1(inner2, at, T, std::size_t)

private:
  typedef typename target_t::iterator                  iter_t;
  typedef typename target_t::const_iterator            const_iter_t;

  typedef typename target_t::reverse_iterator          r_iter_t;
  typedef typename target_t::const_reverse_iterator    const_r_iter_t;

public:
  typedef member_iterator<iter_t, Accessor>            iterator;
  typedef member_iterator<const_iter_t, Accessor>      const_iterator;

  typedef member_iterator<r_iter_t, Accessor>          r_iterator;
  typedef member_iterator<const_r_iter_t, Accessor>    const_r_iterator;

  typedef inner_reference                              reference;
  typedef inner_reference                              const_reference;

  iterator begin()
  {
    return iterator(Accessor::invoke(&target_t::begin), *this);
  }

  iterator end()
  {
    return iterator(Accessor::invoke(&target_t::end), *this);
  }

  r_iterator rbegin()
  {
    return r_iterator(Accessor::invoke(&target_t::rbegin), *this);
  }

  r_iterator rend()
  {
    return r_iterator(Accessor::invoke(&target_t::rend), *this);
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

