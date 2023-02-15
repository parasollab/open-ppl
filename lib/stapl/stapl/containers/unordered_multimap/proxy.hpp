/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_UNORDERED_MULTIMAP_PROXY_HPP
#define STAPL_CONTAINERS_UNORDERED_MULTIMAP_PROXY_HPP

#include "unordered_multimap_fwd.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization used when the type is an unordered_map. Refer
///   to @ref unordered_map for proper use of the container itself.
/// @ingroup punorderedmultimap
////////////////////////////////////////////////////////////////////////
template<typename Key, typename Mapped, typename Hash, typename Pred,
         typename PS, typename M, typename Traits, typename Accessor>
class proxy<stapl::unordered_multimap<Key, Mapped, Hash, Pred, PS, M, Traits>,
        Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef stapl::unordered_multimap<
    Key, Mapped, Hash, Pred, PS, M, Traits
  >                                                         target_t;
  typedef typename target_t::iterator                       iter_t;

public:
  typedef Key                                               key_type;
  typedef typename target_t::mapped_type                    mapped_type;
  typedef typename target_t::size_type                      size_type;
  typedef typename target_t::gid_type                       gid_type;
  typedef typename target_t::gid_type                       index_type;
  typedef typename target_t::value_type                     value_type;
  typedef typename target_t::stored_type                    stored_type;
  typedef typename target_t::reference                      reference;
  typedef typename target_t::second_reference               second_reference;
  typedef typename target_t::domain_type                    domain_type;
  typedef iter_t                                            iterator;

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

  iterator make_iterator(gid_type const& gid)
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

#endif // STAPL_CONTAINERS_UNORDERED_MULTIMAP_PROXY_HPP
