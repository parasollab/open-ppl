/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_UNORDERED_MAP_PROXY_HPP
#define STAPL_CONTAINERS_UNORDERED_MAP_PROXY_HPP

#include "unordered_map_fwd.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization used when the type is an unordered_map. Refer
///   to @ref unordered_map for proper use of the container itself.
/// @ingroup punorderedmap
////////////////////////////////////////////////////////////////////////
template<typename Key, typename Mapped, typename ...OptionalParams,
  typename Accessor>
class proxy<stapl::unordered_map<Key, Mapped, OptionalParams...>,
        Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef stapl::unordered_map<Key, Mapped, OptionalParams...> target_t;
  typedef stapl::map_view<target_t>                            target_view_t;
  typedef typename target_t::iterator                          iter_t;

public:
  typedef Key                                               key_type;
  typedef typename target_t::mapped_type                    mapped_type;
  typedef typename target_t::size_type                      size_type;
  typedef typename target_t::gid_type                       gid_type;
  typedef typename target_t::gid_type                       index_type;
  typedef typename target_t::value_type                     value_type;
  typedef typename target_t::stored_type                    stored_type;
  typedef typename target_t::key_equal                      key_equal;
  typedef typename target_t::hasher                         hasher;
  typedef typename target_t::reference                      reference;
  typedef typename target_t::const_reference                const_reference;
  typedef typename target_t::second_reference               second_reference;
  typedef typename target_t::domain_type                    domain_type;
  typedef typename target_t::view_type                      view_type;
  typedef typename target_view_t::map_func_type             map_func_type;
  typedef typename target_view_t::view_container_type       view_container_type;
  typedef iter_t                                            iterator;
  typedef typename target_t::const_iterator                 const_iterator;

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

  size_t version(void) const
  {
    return Accessor::const_invoke(&target_t::version);
  }

  second_reference operator[](key_type const& k)
  {
    return Accessor::invoke(&target_t::operator[], k);
  }

  void clear(void)
  {
    Accessor::invoke(&target_t::clear);
  }

  value_type get_element(gid_type const& gid)
  {
    return Accessor::invoke(&target_t::get_element, gid);
  }

  reference make_reference(key_type const& k)
  {
    return Accessor::invoke(&target_t::make_reference,k);
  }

  size_type size() const
  {
    return Accessor::const_invoke(&target_t::size);
  }

  bool empty() const
  {
    return Accessor::const_invoke(&target_t::empty);
  }

  domain_type domain() const
  {
    return Accessor::const_invoke(&target_t::domain);
  }

  void define_type(typer& t)
  {
    t.base<Accessor>(*this);
  }
}; // struct proxy

} // namespace stapl

#endif // STAPL_CONTAINERS_UNORDERED_MAP_PROXY_HPP
