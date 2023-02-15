/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_OPERATIONS_MULTI_DIMENSIONAL_SUBSCRIPT_HPP
#define STAPL_VIEWS_OPERATIONS_MULTI_DIMENSIONAL_SUBSCRIPT_HPP

#include <stapl/containers/type_traits/dimension_traits.hpp>
#include <stapl/containers/partitions/block_partition.hpp>

namespace stapl {

namespace view_operations {

//////////////////////////////////////////////////////////////////////
/// @brief Defines the operations to access elements using multiple
///   dimensions via operator().
//////////////////////////////////////////////////////////////////////
template<typename Derived>
class multi_dimensional_subscript
{
private:
  typedef typename view_traits<Derived>::index_type    index_t;
  typedef typename view_traits<Derived>::domain_type   domain_t;
  typedef typename view_traits<Derived>::container     container_t;
  typedef typename view_traits<Derived>::reference     reference_t;

public:
  typedef typename dimension_traits<container_t>::type dimension_type;

private:
  Derived const& derived() const
  {
    return static_cast<Derived const&>(*this);
  }

public:
  typename domain_t::size_type dimensions() const
  {
    return derived().domain().dimensions();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Access an element of a multidimensional container using a tuple
  ///        of indices.
  //////////////////////////////////////////////////////////////////////
  reference_t operator[](index_t const& index) const
  {
    return make_reference(index);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Alternative to operator[] that allows accessing an element
  ///        of a multidimensional container using its indices directly,
  ///        without the need to form a tuple of them first.
  //////////////////////////////////////////////////////////////////////
  template<typename ...Indices>
  reference_t operator()(Indices... indices) const
  {
    return make_reference(indices...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  /// @brief Constructs a reference to the element at the given @c index.
  //////////////////////////////////////////////////////////////////////
  reference_t make_reference(index_t const& index) const
  {
    stapl_assert(derived().domain().contains(index),
                 "index out of view domain boundary\n");

    return detail::make_reference<Derived>()(derived(), index);
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  /// @brief Constructs a reference to the element at the given @c indices.
  //////////////////////////////////////////////////////////////////////
  template<typename Index0, typename Index1, typename ...Indices>
  reference_t
  make_reference(Index0 index0, Index1 index1, Indices... indices) const
  {
    stapl_assert(derived().domain().contains(index0, index1, indices...),
                 "index out of view domain boundary\n");

    return detail::make_reference<Derived>()(derived(), index0, index1,
      indices...);
  }

}; // class multi_dimensional_subscript

} // namespace view_operations

} // namespace stapl

#endif // STAPL_VIEWS_OPERATIONS_MULTI_DIMENSIONAL_SUBSCRIPT_HPP
