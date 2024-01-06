/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_CONST_LOCAL_ACCESSOR_HPP
#define STAPL_CONTAINERS_CONST_LOCAL_ACCESSOR_HPP

#include <type_traits>

#include <stapl/views/proxy.h>
#include <stapl/views/iterator/iterator_facade.h>
#include <stapl/containers/type_traits/is_container.hpp>
#include <stapl/views/proxy/accessor_base.hpp>
#include <stapl/containers/type_traits/container_traits.hpp>
#include <stapl/utility/invoke_arg.hpp>
#include <stapl/views/proxy/accessor_traits.hpp>

namespace stapl {

template<typename C>
class const_local_accessor;

template<typename C>
struct accessor_traits<const_local_accessor<C> >
{
  using is_localized = std::true_type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Accessor for local proxies over base containers.
///
/// @tparam C Type of the container
/// @tparam Iterator Type used by the base container for its iterators
///
/// @see proxy
/// @todo Fix access control of data members
//////////////////////////////////////////////////////////////////////
template<typename C>
class const_local_accessor
  : public accessor_base<
      typename container_traits<C>::value_type,
      const_local_accessor<C>
   >
{
private:
  typedef typename container_traits<C>::container_type::const_iterator
    const_iterator;
  friend class accessor_core_access;

public: //FIXME private:
  const_iterator  m_itr;
  C const*        m_container;

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Always returns true.
  //////////////////////////////////////////////////////////////////////
  bool is_local() const
  {
    return true;
  }

public:
  typedef typename container_traits<C>::value_type               value_type;
  typedef typename container_traits<C>::gid_type                 gid_type;
  typedef gid_type                                               index_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a local accessor with the container pointer and
  /// an iterator to the value for which to create the reference
  /// @param container Pointer to the container
  /// @param it Iterator to the element
  //////////////////////////////////////////////////////////////////////
  const_local_accessor(C const* container, const_iterator const& it)
    : m_itr(it), m_container(container)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an accessor for a null reference
  //////////////////////////////////////////////////////////////////////
  const_local_accessor(null_reference const&)
    : m_itr(NULL), m_container(NULL)
  { }

  const_local_accessor(const_local_accessor const& other)
    : m_itr(other.m_itr), m_container(other.m_container)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether or not this is a null accessor
  //////////////////////////////////////////////////////////////////////
  bool is_null() const
  {
    return m_container == NULL;
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Read the reference by returning a copy of the value pointed
  /// to by the iterator
  //////////////////////////////////////////////////////////////////////
  value_type read() const
  {
    return *m_itr;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the index (GID) of the reference
  //////////////////////////////////////////////////////////////////////
  gid_type index() const
  {
    if (m_container==NULL)
      return index_bounds<gid_type>::invalid();

    const size_t offset = std::distance(m_container->container().begin(),m_itr);
    const gid_type first = m_container->domain().first();

    return m_container->domain().advance(first, offset);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the reference element
  /// and returns the result.
  /// @param f Functor to apply. The function object must export a nested
  /// trait for result_type and its function operator must be declared const
  /// @return Result of applying the functor to the element
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  typename F::result_type apply_get(F const& f) const
  {
    return f(*m_itr);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Serialization of a local accessor is not supported, as it must
  /// first be promoted to a global reference before shipping
  //////////////////////////////////////////////////////////////////////
  void define_type(typer&)
  {
    stapl_assert(false,"This should not be called\n");
  }
}; // struct local_accessor

} // namespace stapl

#endif // STAPL_CONTAINERS_CONST_LOCAL_ACCESSOR_HPP
