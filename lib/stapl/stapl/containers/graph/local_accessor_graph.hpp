/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_LOCAL_ACCESSOR_GRAPH_HPP
#define STAPL_CONTAINERS_GRAPH_LOCAL_ACCESSOR_GRAPH_HPP

#include <stapl/containers/graph/const_local_accessor_graph.hpp>
#include <stapl/containers/iterators/local_accessor.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/type_traits/container_wrapper_ref.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor creates a reference either as a proxy or a view, based
///   on @ref use_view_as_reference metafunction.
/// @tparam T The value type of the outer container, possibly another container.
/// @tparam The Outer container type.
/// @tparam GID Gid type of the outer container.
///
/// Primary template constructs a proxy and returns it as the reference for the
/// iterator dereference. The sole specialization instead heap allocates the
/// proxy and uses it as the container for a view (the type of which is
/// determined by @ref container_traits.
//////////////////////////////////////////////////////////////////////
template<typename T, typename Ref, bool = is_container_wrapper_ref<T>::value>
struct local_property_reference_constructor
{
public:
  typedef Ref                 result_type;

  result_type operator()(Ref prop_ref) const
  { return prop_ref; }
};

template<template<typename> class T, typename C, typename Ref>
struct local_property_reference_constructor<T<C>, Ref, true>
{
public:
  typedef typename container_traits<C>::
    template construct_view<C>::type     result_type;

  result_type operator()(Ref prop_ref) const
  { return result_type(prop_ref.get()); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Local accessor for pGraph's vertices.
/// @ingroup pgraphDistObj
/// Derives from @ref local_accessor and adds methods for property.
/// @tparam C type of the pGraph.
//////////////////////////////////////////////////////////////////////
template<typename C>
class local_accessor_graph
  : public local_accessor<C>
{
private:
  typedef typename container_traits<C>::container_type::iterator iterator;
  typedef local_accessor<C>                                      base_type;

  typedef typename is_container<typename C::value_type>::type
    is_composed_container_t;

public:
  typedef typename C::value_type                     value_type;
  typedef typename base_type::gid_type               gid_type;

  typedef local_property_reference_constructor<
    typename value_type::property_type,
    typename value_type::property_reference>         referencer_t;
  typedef typename referencer_t::result_type         property_reference;


  typedef const_local_property_reference_constructor<
    typename value_type::property_type,
    typename value_type::const_property_reference>   const_referencer_t;
  typedef typename const_referencer_t::result_type   const_property_reference;

  local_accessor_graph(C* container, iterator const& it)
    : base_type(container, it)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc local_accessor::local_accessor(null_reference const&)
  //////////////////////////////////////////////////////////////////////
  local_accessor_graph(null_reference const& nr)
    : base_type(nr)
  { }

  local_accessor_graph(local_accessor_graph const& other)
    : base_type(other)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Read the reference by returning a true reference of the value
  ///   pointed to by the iterator
  /// @todo Remove this method.  Access to a reference of element needs
  ///   to be restricted to the base container. The container manager can
  ///   forward requests via apply_set/get to the appropriate base container.
  //////////////////////////////////////////////////////////////////////
  value_type& ref()
  {
    return *(this->m_itr);
  }

  value_type const& ref() const
  {
    return *(this->m_itr);
  }

  property_reference property()
  {
    return referencer_t()(this->ref().property());
  }

  const_property_reference property() const
  {
    return const_referencer_t()(this->ref().property());
  }

  const_property_reference const_property() const
  {
    return const_referencer_t()(this->ref().property());
  }

  gid_type index() const
  {
    return base_type::index();
  }

private:
  void define_type(typer&);
}; // struct local_accessor_graph

} // namespace stapl

#endif // STAPL_CONTAINERS_GRAPH_LOCAL_ACCESSOR_GRAPH_HPP
