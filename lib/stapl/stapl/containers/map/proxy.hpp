/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MAP_PROXY_HPP
#define STAPL_CONTAINERS_MAP_PROXY_HPP

#include "map_fwd.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization used when the type is an map. Refer to @ref
/// map for proper use of the container itself.
/// @ingroup pmap
////////////////////////////////////////////////////////////////////////
template<typename Key, typename T, typename ...OptionalParams,
         typename Accessor>
class proxy<stapl::map<Key, T, OptionalParams...>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef stapl::map<Key, T, OptionalParams...>             target_t;
  typedef typename target_t::iterator                       iter_t;

public:
  typedef Key                                               key_type;
  typedef typename target_t::mapped_type                    mapped_type;
  typedef typename target_t::size_type                      size_type;
  typedef typename target_t::gid_type                       gid_type;
  typedef typename target_t::gid_type                       index_type;
  typedef typename target_t::value_type                     value_type;
  typedef typename target_t::reference                      reference;
  typedef typename target_t::const_reference                const_reference;
  typedef typename target_t::second_reference               second_reference;
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

  location_type get_location_id(void) const
  {
    return Accessor::const_invoke(&target_t::get_location_id);
  }

  size_t get_num_locations(void) const
  {
    return Accessor::const_invoke(&target_t::get_num_locations);
  }

  locality_info locality(gid_type gid)
  {
    return Accessor::invoke(&target_t::locality, gid);
  }

  rmi_handle::reference get_rmi_handle(void)
  {
    return Accessor::invoke(&target_t::get_rmi_handle_reference);
  }

  second_reference operator[](key_type const& gid) const
  {
    return Accessor::invoke(&target_t::operator[], gid);
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
}; // struct proxy

} // namespace stapl

#endif // STAPL_CONTAINERS_MAP_PROXY_HPP
