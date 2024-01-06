/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_UNORDERED_MULTISET_PROXY_HPP
#define STAPL_CONTAINERS_UNORDERED_MULTISET_PROXY_HPP

#include "unordered_multiset_fwd.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization used when the type is an unordered multiset.
///   Refer to @ref unordered multiset for proper use of the container
///   itself.
/// @ingroup punorderedmultiset
////////////////////////////////////////////////////////////////////////
template<typename Key, typename ...OptionalParams, typename Accessor>
class proxy<stapl::unordered_multiset<Key, OptionalParams...>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef stapl::unordered_multiset<Key, OptionalParams...> target_t;
  typedef stapl::set_view<target_t>                         target_view_t;
  typedef typename target_t::iterator                       iter_t;

public:
  typedef Key                                               key_type;
  typedef Key                                               mapped_type;
  typedef typename target_t::size_type                      size_type;
  typedef typename target_t::gid_type                       gid_type;
  typedef typename target_t::gid_type                       index_type;
  typedef typename target_t::value_type                     value_type;
  typedef typename target_t::stored_type                    stored_type;
  typedef typename target_t::key_equal                      key_equal;
  typedef typename target_t::hasher                         hasher;
  typedef typename target_t::reference                      reference;
  typedef typename target_t::const_reference                const_reference;
  typedef typename target_t::domain_type                    domain_type;
  typedef typename target_t::view_type                      view_type;
  typedef typename target_view_t::map_func_type             map_func_type;
  typedef typename target_view_t::view_container_type       view_container_type;
  typedef iter_t                                            iterator;
  typedef iterator                                          const_iterator;

  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t() const
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

  void erase(key_type const& key)
  {
    Accessor::invoke(&target_t::erase, key);
  }

  size_t erase_sync(key_type const& key)
  {
    return Accessor::invoke(&target_t::erase_sync, key);
  }

  void clear(void)
  {
    Accessor::invoke(&target_t::clear);
  }

  size_t count(key_type const& key) const
  {
    return Accessor::const_invoke(&target_t::count, key);
  }

  iterator make_iterator(gid_type const& gid) const
  {
    return Accessor::const_invoke(&target_t::make_iterator, gid);
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

  bool validate(void) const
  {
    return true;
  }

  void define_type(typer& t)
  {
    t.base<Accessor>(*this);
  }
}; // class proxy

} // namespace stapl

#endif // STAPL_CONTAINERS_UNORDERED_MULTISET_PROXY_HPP
