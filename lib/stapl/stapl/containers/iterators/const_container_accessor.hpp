/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_CONST_CONTAINER_ACCESSOR_HPP
#define STAPL_CONTAINERS_CONST_CONTAINER_ACCESSOR_HPP

#include <stapl/views/proxy/accessor.hpp>
#include <stapl/views/proxy/accessor_base.hpp>
#include <stapl/containers/type_traits/container_traits.hpp>
#include <boost/utility/result_of.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Const_accessor for global proxies over pContainers.
///
/// @tparam Container Type of the container
///
/// @see proxy
/// @bug Using base_accessor for read and write fails at the moment for some
/// point. Resolve whatever inconsistency is causing this.
//////////////////////////////////////////////////////////////////////
template <typename Container>
class const_container_accessor
  : public accessor_base<
            typename container_traits<Container>::value_type,
            const_container_accessor<Container> >
{
public:
  typedef Container                                         container_type;
  typedef typename container_traits<Container>::gid_type    index_type;
  typedef typename container_traits<Container>::value_type  value_type;

protected: // private
  container_type const* m_container;
  index_type            m_index;

private:
  template <typename Derived, typename A, typename C, typename D>
  friend class iterator_facade;

  friend class accessor_core_access;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether or not this is a null accessor
  //////////////////////////////////////////////////////////////////////
  bool is_null(void) const
  {
    if (m_index!=index_bounds<index_type>::invalid())
      return false;

    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Serialization for the accessor
  //////////////////////////////////////////////////////////////////////
  void define_type(typer& t)
  {
    t.member(m_container);
    t.member(m_index);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_accessor for a null reference
  //////////////////////////////////////////////////////////////////////
  const_container_accessor(void)
    : m_container(NULL),
      m_index(index_bounds<index_type>::invalid())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_accessor for a null reference
  //////////////////////////////////////////////////////////////////////
  explicit const_container_accessor(null_reference const&)
    : m_container(NULL),
      m_index(index_bounds<index_type>::invalid())
  { }

  const_container_accessor(const_container_accessor const& other)
    : m_container(other.m_container), m_index(other.m_index)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a global const_accessor with the container and index
  /// @param container Pointer to the container
  /// @param index Index of the reference
  //////////////////////////////////////////////////////////////////////
  const_container_accessor(container_type const* container,
                           index_type const& index)
    : m_container(container), m_index(index)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the index of the reference
  //////////////////////////////////////////////////////////////////////
  index_type index(void) const
  {
    return m_index;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return whether or not this is a local or remote reference
  //////////////////////////////////////////////////////////////////////
  bool is_local(void) const
  {
    return m_container->is_local(m_index);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Read the reference by returning a copy of the value in the
  /// container at the index
  //////////////////////////////////////////////////////////////////////
  value_type read(void) const
  {
    return m_container->get_element(m_index);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a reference by returning a copy of the value in the
  /// container at the index.
  //////////////////////////////////////////////////////////////////////
  value_type ref(void) const
  {
    return read();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the reference element
  /// and returns the result.
  /// @param f Functor to apply. The function object must export a nested
  /// trait for result_type and its function operator must be declared const.
  /// @return Result of applying the functor to the element
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  typename boost::result_of<F(value_type)>::type
  apply_get(F const& f) const
  {
    return m_container->apply_get(m_index, f);
  }
};

}

#endif // STAPL_CONTAINERS_CONST_CONTAINER_ACCESSOR_HPP
