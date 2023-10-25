/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_LOCAL_ACCESSOR_HPP
#define STAPL_CONTAINERS_LOCAL_ACCESSOR_HPP

#include <type_traits>

#include <stapl/views/proxy.h>
#include <stapl/views/iterator/iterator_facade.h>
#include <stapl/containers/type_traits/is_container.hpp>
#include <stapl/containers/type_traits/container_traits.hpp>
#include <stapl/containers/type_traits/index_bounds.hpp>
#include <stapl/views/proxy/accessor_traits.hpp>

namespace stapl {

template<typename C>
class local_accessor;


template<typename C>
struct accessor_traits<local_accessor<C>>
{
  using is_localized = std::true_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Type metafunction to allow base containers to specify a
///   different iterator type than their basic reflected type.  Used in
///   matrix base container when immediate iterator is a row iterator.
//////////////////////////////////////////////////////////////////////
template<typename C>
struct local_accessor_traits
{
  using iterator = typename container_traits<C>::container_type::iterator;
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
class local_accessor
{
private:
  using iterator = typename local_accessor_traits<C>::iterator;

  friend class accessor_core_access;

public: //FIXME private:
  iterator  m_itr;
  C*        m_container;

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Always returns true.
  //////////////////////////////////////////////////////////////////////
  bool is_local() const
  {
    return true;
  }

public:
  using value_type = typename container_traits<C>::value_type;
  using gid_type   = typename container_traits<C>::gid_type;
  using index_type = gid_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a local accessor with the container pointer and
  /// an iterator to the value for which to create the reference
  /// @param container Pointer to the container
  /// @param it Iterator to the element
  //////////////////////////////////////////////////////////////////////
  local_accessor(C* container, iterator const& it)
    : m_itr(it), m_container(container)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an accessor for a null reference
  //////////////////////////////////////////////////////////////////////
  local_accessor(null_reference const&)
    : m_itr(NULL), m_container(NULL)
  { }

  local_accessor(local_accessor const& other)
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
  /// @brief Write the reference by setting the value pointed to by the
  /// iterator to a given value
  /// @param val An element that can be convertible to the reference's
  /// value_type that is to be written
  //////////////////////////////////////////////////////////////////////
  void write(value_type const& val) //const
  {
    *m_itr = val;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Write the reference by forwarding the given value to the
  /// element pointed to by the iterator
  /// @param val An rvalue reference to an element that can be converted
  //  to the reference's value_type that is to be written
  //////////////////////////////////////////////////////////////////////
  void write(value_type&& val)
  {
    *m_itr = std::move(val);
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
  /// @brief Applies an arbitrary functor to the reference element.
  /// @param f Functor to apply. The function operator of the functor
  /// must be declared const.
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  void apply_set(F const& f) const
  {
    f(*m_itr);
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

  template<typename Class, typename... Args>
  void invoke(void (Class::* const pmf)(Args...),
              typename std::decay<Args>::type const&... args) const

  {
    ((*m_itr).*pmf)(args...);
  }

  template<typename Class, typename Rtn, typename... Args>
  Rtn invoke(Rtn (Class::* const pmf)(Args...),
             typename std::decay<Args>::type const&... args) const
  {
    return ((*m_itr).*pmf)(args...);
  }

  template<typename Class, typename Rtn, typename... Args>
  Rtn const_invoke(Rtn (Class::* const pmf)(Args...) const,
                   typename std::decay<Args>::type const&... args) const

  {
    return ((*m_itr).*pmf)(args...);
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

#endif // STAPL_CONTAINERS_LOCAL_ACCESSOR_HPP
