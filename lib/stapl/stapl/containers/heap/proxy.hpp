/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_HEAP_PROXY_HPP
#define STAPL_CONTAINERS_HEAP_PROXY_HPP

#include "heap_fwd.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for the heap container.
/// @ingroup pheapDistObj
/// @tparam T Type of the stored elements in the container.
/// @tparam Comp The comparator used to inferred an ordering between
///         elements.
/// @tparam P Partition strategy that defines how to partition
///         the original domain into subdomains.
/// @tparam M Mapper that defines how to map the subdomains produced
///         by the partition to locations.
/// @tparam Traits A traits class that defines customizable components
///         of the heap container.
/// @tparam Acessor A function used to modify heap variables.
////////////////////////////////////////////////////////////////////////
template <typename T, typename Comp, typename P,
          typename M, typename Traits, typename Accessor>
class proxy<stapl::heap<T, Comp, P, M, Traits>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef stapl::heap<T, Comp, P, M, Traits>   target_t;
  typedef typename target_t::iterator          iter_t;

public:
  typedef typename target_t::size_type         size_type;
  typedef typename target_t::gid_type          gid_type;
  typedef typename target_t::gid_type          index_type;
  typedef typename target_t::value_type        value_type;
  typedef typename target_t::reference         reference;
  typedef typename target_t::domain_type       domain_type;
  typedef iter_t                               iterator;

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

  iterator make_iterator(gid_type const& gid) const
  {
    return Accessor::invoke(&target_t::make_iterator, gid);
  }

  iterator make_iterator(domain_type const& domain,
                         gid_type const& gid)
  {
    return Accessor::invoke(&target_t::make_iterator,domain,gid);
  }

  void set_element(gid_type gid, value_type const& value)
  {
    Accessor::invoke(&target_t::set_element, gid, value);
  }

  reference make_reference(index_type const& idx)
  {
    return Accessor::invoke(&target_t::make_reference,idx);
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

#endif /* STAPL_CONTAINERS_HEAP_PROXY_HPP */
