/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_OPERATIONS_SUBSCRIPT_HPP
#define STAPL_VIEWS_OPERATIONS_SUBSCRIPT_HPP

#include <stapl/views/operations/make_reference.hpp>

namespace stapl {

namespace view_operations {

//////////////////////////////////////////////////////////////////////
/// @brief Defines the operations to access the elements of a one-dimensional
///        container using the random access operator (@c []).
//////////////////////////////////////////////////////////////////////
template<typename Derived>
class subscript
{
private:
  using index_t = typename view_traits<Derived>::index_type;

public:
  using reference_t = typename view_traits<Derived>::reference;

private:
  Derived const& derived() const
  {
    return static_cast< Derived const&>(*this);
  }

public:
  /// @name View Subscript Operations
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief The bracket operator is the basic access method.
  //////////////////////////////////////////////////////////////////////
  reference_t operator[](index_t const& index) const
  {
    return make_reference(index);
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

  /// @}
}; // class subscript

} // namespace view_operations

} // namespace stapl

#endif // STAPL_VIEWS_OPERATIONS_SUBSCRIPT_HPP
