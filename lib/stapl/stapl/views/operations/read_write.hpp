/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_OPERATIONS_READ_WRITE_HPP
#define STAPL_VIEWS_OPERATIONS_READ_WRITE_HPP

#include <stapl/views/view_traits.hpp>

namespace stapl {

namespace view_operations {


//////////////////////////////////////////////////////////////////////
/// @brief Defines operations to read values from the underlying
///        container
//////////////////////////////////////////////////////////////////////
template<typename Derived>
class read
{
private:
  typedef typename view_traits<Derived>::index_type            index_t;
  typedef typename view_traits<Derived>::reference             reference_t;
  typedef typename view_traits<Derived>::value_type            value_type;

  const Derived& derived() const
  {
    return static_cast<const Derived&>(*this);
  }

public:
  /// @name View Read Operations
  // @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Get the element @c index from the container
  /// @param index of element to get
  //////////////////////////////////////////////////////////////////////
  value_type
  get_element(index_t const& index) const
  {
    stapl_assert(derived().domain().contains(index),
                 "index out of view domain boundary\n");
    typename Derived::map_func_type mf = derived().mapfunc();
    return derived().container().get_element(mf(index));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Get the element @c index from the container
  /// @param index of element to get
  //////////////////////////////////////////////////////////////////////
  future<value_type>
  get_element_split(index_t const& index) const
  {
    stapl_assert(derived().domain().contains(index),
                 "index out of view domain boundary\n");
    typename Derived::map_func_type mf = derived().mapfunc();
    return derived().container().get_element_split(mf(index));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies the provided function to the value referenced for
  ///        the given index and returns the result of the operation.
  ///
  /// @param index of element to apply the function
  /// @param f function to apply
  /// @return result of evaluate the function f on the value
  ///         referenced for the index
  //////////////////////////////////////////////////////////////////////
  template<class Functor>
  typename Functor::result_type
  apply_get(index_t const& index, Functor f)
  {
    stapl_assert(derived().domain().contains(index),
                 "index out of view domain boundary\n");
    return derived().container().apply_get(derived().mapfunc()(index), f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc read::apply_get(index_t const&, Functor)
  //////////////////////////////////////////////////////////////////////
  template<class Functor>
  typename Functor::result_type
  apply_get(index_t const& index, Functor f) const
  {
    stapl_assert(derived().domain().contains(index),
                 "index out of view domain boundary\n");
    return derived().container().apply_get(derived().mapfunc()(index), f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  /// @brief Applies the provided function the the value referenced
  ///        for the given index returning the resulting value
  ///
  /// @param index of element to apply the function
  /// @param f function to apply
  /// @return result of evaluate the function f on the value
  ///         referenced for the index
  /// @todo Verify if this is a dead code and remove it.
  //////////////////////////////////////////////////////////////////////
  template<class Functor>
  typename Functor::result_type
  apply_get_local(index_t const& index, Functor f)
  {
    stapl_assert(derived().domain().contains(index),
                 "index out of view domain boundary\n");
    return derived().container().apply_get_local(derived().mapfunc()(index), f);
  }

  /// @}
}; //class read


// Write Operation
template<typename Derived>
class write
{
private:
  typedef typename view_traits<Derived>::value_type value_t;
  typedef typename view_traits<Derived>::index_type index_t;

  Derived const& derived() const
  {
    return static_cast<const Derived&>(*this);
  }

public:
  /// @name View Write Operations
  // @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the element @c index in the container to value @c value
  /// @param index Index of element to set.
  /// @param value New value to set.
  //////////////////////////////////////////////////////////////////////
  void set_element(index_t const& index, value_t const& value)
  {
    stapl_assert(derived().domain().contains(index),
                 "index out of view domain boundary\n");
    derived().container().set_element(derived().mapfunc()(index), value);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies the provided function to the value referenced for
  ///        the given index and mutates the element with the
  ///        resulting value
  ///
  /// @param index of element to apply the function
  /// @param f function to apply
  //////////////////////////////////////////////////////////////////////
  template<class Functor>
  void apply_set(index_t const& index, Functor f)
  {
    stapl_assert(derived().domain().contains(index),
                 "index out of view domain boundary\n");
    derived().container().apply_set(derived().mapfunc()(index), f);
  }

  /// @}

}; //class write


// Read-Write Operation
template<typename Derived>
class readwrite
  : public read<Derived>,
    public write<Derived>
{ };

} // view_operations namespace

} // stapl namespace

#endif /* STAPL_VIEWS_OPERATIONS_READ_WRITE_HPP */
