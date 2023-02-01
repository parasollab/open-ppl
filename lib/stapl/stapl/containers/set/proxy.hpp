/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SET_PROXY_HPP
#define STAPL_CONTAINERS_SET_PROXY_HPP

#include "set_fwd.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization used when the type is an set. Refer to @ref
/// set for proper use of the container itself.
/// @ingroup pset
////////////////////////////////////////////////////////////////////////
template <typename Key, typename ...OptionalParams, typename Accessor>
class proxy<stapl::set<Key, OptionalParams...>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef stapl::set<Key, OptionalParams...>                target_t;
  typedef typename target_t::iterator                       iter_t;

public:
  typedef Key                                               key_type;
  typedef typename target_t::size_type                      size_type;
  typedef typename target_t::gid_type                       gid_type;
  typedef typename target_t::gid_type                       index_type;
  typedef typename target_t::value_type                     value_type;
  typedef typename target_t::reference                      reference;
  typedef typename target_t::const_reference                const_reference;
  typedef typename target_t::domain_type                    domain_type;
  typedef iter_t                                            iterator;

  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t(void) const
  {
    return Accessor::read();
  }

  proxy const& operator=(proxy const& rhs)
  {
    Accessor::write(rhs);
    return *this;
  }

  proxy const& operator=(target_t const& rhs)
  {
    Accessor::write(rhs);
    return *this;
  }

  void insert(key_type const& key)
  {
    Accessor::invoke(&target_t::insert, key);
  }

  size_t version(void) const
  {
    return Accessor::const_invoke(&target_t::version);
  }

  size_type size(void) const
  {
    return Accessor::const_invoke(&target_t::size);
  }

  bool empty(void) const
  {
    return Accessor::const_invoke(&target_t::empty);
  }

  domain_type domain(void) const
  {
    return Accessor::const_invoke(&target_t::domain);
  }

  iterator make_iterator(gid_type const& gid) const
  {
    return Accessor::invoke(&target_t::make_iterator, gid);
  }

  value_type get_element(gid_type const& gid)
  {
    return Accessor::invoke(&target_t::get_element, gid);
  }

  reference make_reference(index_type const& idx)
  {
    return Accessor::invoke(&target_t::make_reference,idx);
  }

  void define_type(typer& t)
  {
    t.base<Accessor>(*this);
  }
}; // class proxy

} // namespace stapl

#endif // STAPL_CONTAINERS_SET_PROXY_HPP
