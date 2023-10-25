/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_STATIC_ARRAY_PROXY_HPP
#define STAPL_CONTAINERS_STATIC_ARRAY_PROXY_HPP

#include "static_array_fwd.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization used when the type is a static_array. Refer to @ref
/// static_array for proper use of the container itself.
/// @ingroup parrayDistObj
///
/// @see static_array proxy
////////////////////////////////////////////////////////////////////////
template <typename T, typename Accessor>
class proxy<stapl::static_array<T>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef stapl::static_array<T>               target_t;
  typedef typename target_t::iterator          iter_t;
  typedef typename target_t::const_iterator    const_iter_t;

public:
  typedef typename target_t::size_type         size_type;
  typedef typename target_t::gid_type          gid_type;
  typedef typename target_t::gid_type          index_type;
  typedef typename target_t::value_type        value_type;
  typedef typename target_t::reference         reference;
  typedef typename target_t::const_reference   const_reference;
  typedef typename target_t::domain_type       domain_type;
  typedef iter_t                               iterator;
  typedef const_iter_t                         const_iterator;

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

  size_type size(void) const
  {
    return Accessor::const_invoke(&target_t::size);
  }

  void resize(size_t size)
  {
    Accessor::invoke(&target_t::resize, size);
  }

  bool empty(void) const
  {
    return Accessor::const_invoke(&target_t::empty);
  }

  reference operator[](size_type gid)
  {
    return Accessor::invoke(&target_t::operator[], gid);
  }

  const_reference operator[](size_type gid) const
  {
    return Accessor::const_invoke(&target_t::operator[], gid);
  }

  domain_type domain(void) const
  {
    return Accessor::const_invoke(&target_t::domain);
  }

  iterator make_iterator(gid_type const& gid)
  {
    return Accessor::invoke(&target_t::make_iterator, gid);
  }

  const_iterator make_iterator(gid_type const& gid) const
  {
    return Accessor::const_invoke(&target_t::make_iterator, gid);
  }

  void set_element(gid_type const& gid, value_type const& value)
  {
    Accessor::invoke(&target_t::set_element, gid, value);
  }

  template<typename View>
  void set_elements(gid_type const& gid, View const& view)
  {
    Accessor::invoke(&target_t::template set_elements<View const&>, gid, view);
  }

  void define_type(typer& t)
  {
    t.base<Accessor>(*this);
  }

}; // struct proxy

} // namespace stapl

#endif // STAPL_CONTAINERS_ARRAY_PROXY_HPP
