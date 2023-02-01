/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_LIST_LIST_PROXY_HPP
#define STAPL_CONTAINERS_LIST_LIST_PROXY_HPP

#include "list_fwd.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for @ref stapl::list.
/// @ingroup plistDistObj
//////////////////////////////////////////////////////////////////////
template <typename T, typename P, typename M, typename Traits,
          typename Accessor>
class proxy<stapl::list<T, P, M, Traits>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef stapl::list<T, P, M, Traits>         target_t;
  typedef typename target_t::iterator          iter_t;

public:
  typedef typename target_t::size_type         size_type;
  typedef typename target_t::gid_type          gid_type;
  typedef typename target_t::gid_type          index_type;
  typedef typename target_t::value_type        value_type;
  typedef typename target_t::reference         reference;
  typedef typename target_t::domain_type       domain_type;
  typedef iter_t                               iterator;

  // typedef typename target_t::loc_dist_metadata  loc_dist_metadata;
  // typedef proxy fast_view_type;

  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t() const
  {
    return Accessor::read();
    // if is composed pcontainer
    // return Accessor::read().get();
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

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs an iterator pointing to the specified element
  ///        in the list.
  /// @param gid Gid of the element.
  ////////////////////////////////////////////////////////////////////////
  iterator make_iterator(gid_type const& gid) const
  {
    return Accessor::invoke(&target_t::make_iterator, gid);
  }

  void set_element(gid_type gid, value_type const& value)
  {
    Accessor::invoke(&target_t::set_element, gid, value);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a reference to the specified element in the list.
  /// @param gid Gid of the element.
  ////////////////////////////////////////////////////////////////////////
  reference make_reference(index_type const& gid)
  {
    return Accessor::invoke(&target_t::make_reference,gid);
  }

  typename target_t::distribution_type* get_distribution()
  {
    return Accessor::invoke(&target_t::get_distribution);
  }

  void define_type(typer& t)
  {
    t.base<Accessor>(*this);
  }

  target_t* get_pointer()
  {
    return Accessor::get_pointer();
  }

}; //struct proxy

} //namespace stapl

#endif /* STAPL_CONTAINERS_LIST_LIST_PROXY_HPP */
