/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_HEAP_VIEW_HPP
#define STAPL_VIEWS_HEAP_VIEW_HPP

#include <stapl/views/core_view.hpp>
#include <stapl/views/mapping_functions/identity.hpp>
#include <stapl/views/operations/sequence.hpp>
#include <stapl/views/type_traits/select_derive.hpp>
#include <stapl/containers/type_traits/container_traits.hpp>
#include <stapl/utility/use_default.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief View for heap container.
/// @tparam C The underlying @ref heap container type.
/// @tparam Dom Type used to represent the GIDs of the elements
///         contained in the view.
/// @tparam MapFunc Function to map from view GIDs to GIDs of the
///         underlying container.
/// @tparam Derived Type of view derived from this view, if any.
//////////////////////////////////////////////////////////////////////
template <typename C,
    typename Dom = typename container_traits<C>::domain_type,
    typename MapFunc = f_ident<typename Dom::index_type>,
    typename Derived = use_default>
class heap_view
  : public core_view<C,Dom,MapFunc>,
    public view_operations::sequence<typename select_derived<
           Derived, heap_view<C, Dom, MapFunc, Derived> >::type>
{
  typedef core_view<C,Dom,MapFunc>                     base_type;

  typedef typename select_derived<
    Derived,
    heap_view<C, Dom, MapFunc, Derived>
  >::type                                              derived_type;

  typedef view_operations::sequence<derived_type>      sequence_op_type;

public:
  STAPL_VIEW_REFLECT_TRAITS(heap_view)

  typedef typename sequence_op_type::iterator          iterator;
  typedef typename sequence_op_type::const_iterator    const_iterator;

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a heap view from a pointer to a container, a domain
  ///        and a mapping function. The view assumes ownership of the
  ///        container and will destroy the container when it is destroyed.
  /// @param vcont Pointer to the container used to forward the operations.
  /// @param dom Domain to be used by the view.
  /// @param mfunc Mapping function to transform view indices to
  ///              container gids.
  //////////////////////////////////////////////////////////////////////
  heap_view(view_container_type* vcont, domain_type const& dom,
            map_func_type mfunc=MapFunc())
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a heap view from a container, a domain
  ///        and a mapping function.
  /// @param vcont Container used to forward the operations.
  /// @param dom Domain to be used by the view.
  /// @param mfunc Mapping function to transform view indices to
  ///        container gids.
  //////////////////////////////////////////////////////////////////////
  heap_view(view_container_type const& vcont, domain_type const& dom,
            map_func_type mfunc=MapFunc())
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a heap view from a pointer to a container.
  ///        The view assumes ownership of the container and will
  ///        destroy the container when it is destroyed.
  /// @param vcont Pointer to the container used to forward the
  ///              operations.
  //////////////////////////////////////////////////////////////////////
  heap_view(view_container_type* vcont)
    : base_type(vcont, stapl::get_domain(*vcont))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a heap view from a container.
  /// @param vcont Container used to forward the operations.
  //////////////////////////////////////////////////////////////////////
  heap_view(view_container_type& vcont)
    : base_type(vcont, stapl::get_domain(vcont))
  { }

  /// @todo Determine why using different derived type is needed.
  template<typename Derived1>
  heap_view(heap_view<C, Dom, MapFunc, Derived1> const& other)
    : base_type(other.get_container(), other.domain(), other.mapfunc())
  { }

  heap_view(heap_view const& other)
    : base_type(other)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator over the first element
  ///        of the container.
  //////////////////////////////////////////////////////////////////////
  iterator begin(void)
  {
    return this->container().make_iterator(
                 this->domain(),this->domain().first());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator over one position past the last element
  ///        in the container.
  //////////////////////////////////////////////////////////////////////
  iterator end(void)
  {
    return this->container().make_iterator(
                 this->domain(),this->domain().open_last());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a const iterator over the first element
  ///        of the container.
  //////////////////////////////////////////////////////////////////////
  const_iterator begin(void) const
  {
    return this->container().make_iterator(
                 this->domain(),this->domain().first());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a const iterator over one position past the last element
  ///        in the container.
  //////////////////////////////////////////////////////////////////////
  const_iterator end(void) const
  {
    return this->container().make_iterator(
                 this->domain(),this->domain().open_last());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to an element of the container
  ///        using its GID.
  /// @param g The GID for which to create a reference.
  /// @return A proxy of the value at gid.
  //////////////////////////////////////////////////////////////////////
  reference make_reference(index_type g) const
  {
    return this->container().make_reference(g);
  }

  //Container operations

  //////////////////////////////////////////////////////////////////////
  /// @brief Insert an element into the container.
  /// @param value Element to be added.
  /// @todo update domain to reflect change in size
  //////////////////////////////////////////////////////////////////////
  void push(value_type const& value)
  {
    this->incr_version();
    this->container().push(value);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a copy of the first element according to the order
  ///        determined by the comparator and removes it from the container.
  /// @todo update domain to reflect change in size
  //////////////////////////////////////////////////////////////////////
  value_type pop(void)
  {
    this->incr_version();
    return this->container().pop();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a copy of the k greatest values existing in the container
  ///        according to the order determined by the comparator.
  /// @param k Number of greatest elements to obtain.
  /// @return A view containing a copy of the k greatest values of the
  ///         container.
  //////////////////////////////////////////////////////////////////////
  array_view<array<value_type> > top_k(size_t const& k)
  {
    return this->container().top_k(k);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes the k greatest values existing in the container
  ///        according to the order determined by the comparator.
  /// @param k Number of elements to remove.
  /// @return A view containing the k greatest values removed from the
  ///         container.
  /// @todo update domain to reflect change in size
  //////////////////////////////////////////////////////////////////////
  array_view<array<value_type> > pop_k(size_t const& k)
  {
    this->incr_version();
    return this->container().pop_k(k);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a copy of the first element according to the order
  ///        determined by the comparator.
  //////////////////////////////////////////////////////////////////////
  value_type top(void)
  {
    return this->container().top();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns true if the container follows the ordering
  ///        inferred by the comparator.
  //////////////////////////////////////////////////////////////////////
  bool is_heap(void)
  {
    return this->container().is_heap();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clear all elements in the container and fill it
  ///        with elements of a view.
  /// @tparam The view used to populate the container.
  /// @todo update domain to reflect change in size
  //////////////////////////////////////////////////////////////////////
  template < typename View>
  void make(View const& v)
  {
    this->incr_version();
    this->container().make(v);
  }


private:

  //////////////////////////////////////////////////////////////////////
  /// @brief Rearranges the container elements using the comparator
  ///        provided in such a way that it forms a heap.
  //////////////////////////////////////////////////////////////////////
  void heapify(void)
  {
    this->container().heapify();
  }


}; //class heap_view

} //namespace stapl

#endif /* STAPL_VIEWS_HEAP_VIEW_HPP */
