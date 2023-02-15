/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PROXY_MAP_H
#define STAPL_PROXY_MAP_H

#include <map>
#include <stapl/views/proxy/proxy.hpp>

namespace stapl {

template<typename BaseIterator, typename ParentAccessor>
class member_iterator;

template<typename Selector, typename... OptionalAccessor>
class member_accessor;

/////////////////////////////////////////////////
/// @todo Macrofy
/// @bug  The proxy is currently only making reference on local data
///       when iterating over the map.
/////////////////////////////////////////////////
template <typename T1, typename T2, typename Accessor>
class proxy<std::map<T1,T2>, Accessor>
  : private Accessor
{
private:
  friend class proxy_core_access;

  typedef std::map<T1,T2>                              target_t;

public:
  typedef typename target_t::size_type                 size_type;
  typedef typename target_t::iterator                  iter_t;
  typedef typename target_t::const_iterator            const_iter_t;
  typedef member_iterator<iter_t, Accessor>            iterator;
  typedef member_iterator<const_iter_t, Accessor>      const_iterator;

  typedef proxy                                        fast_view_type;

  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  {}

  operator target_t() const
  {
    return Accessor::read();
  }

  template<typename Key, typename Value>
  struct get_inner
  {
    Key m_a0;
    get_inner(std::size_t const& a0)
      : m_a0(a0)
    { }
    typedef Value result_type;
    template<typename Outer> result_type operator()(Outer const& v) const
    { return v. operator[](m_a0); }
    template<typename Outer> result_type& operator()(Outer& v) const
    { return v. operator[](m_a0); }
    void define_type(::stapl::typer& t)
    { t.member(m_a0); }
  };

  typedef get_inner<T1,T2> inner_selector_type;
  typedef member_accessor<inner_selector_type, Accessor> inner_accessor;
  typedef proxy<T2, inner_accessor> inner_reference;

  inner_reference operator[](T1 const& ind)
  { return inner_reference( inner_accessor(*this, inner_selector_type(ind))); }

  const_iterator begin() const
  {
    return const_iterator(Accessor::const_invoke(&target_t::begin), *this);
  }

  iterator begin()
  {
    return iterator(Accessor::invoke(&target_t::begin), *this);
  }

  const_iterator end() const
  {
    return const_iterator(Accessor::const_invoke(&target_t::end), *this);
  }

  iterator end()
  {
    return iterator(Accessor::invoke(&target_t::end), *this);
  }

  bool empty() const
  {
    return Accessor::const_invoke(&target_t::empty);
  }

  size_type size() const
  {
    return Accessor::const_invoke(&target_t::size);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Assigns the given value (@p rhs) to the element
  ///        referenced for the proxy.
  ///
  /// Depending on the type of accessor, assigning the value could
  /// generate an asynchronous communication.
  //////////////////////////////////////////////////////////////////////
  proxy const&
  operator=(target_t const& rhs) //const
  {
    Accessor::write(rhs);
    return *this;
  }

  void define_type(typer& t)
  {
    t.base<Accessor>(*this);
  }
};

} // namespace stapl

#endif
