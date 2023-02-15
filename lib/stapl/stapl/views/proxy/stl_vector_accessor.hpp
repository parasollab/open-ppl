/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_PROXY_STL_VECTOR_ACCESSOR_HPP
#define STAPL_VIEWS_PROXY_STL_VECTOR_ACCESSOR_HPP

#include <stapl/views/proxy/accessor_base.hpp>
#include <stapl/runtime/serialization_fwd.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Defines an accessor for std::vector.
///
/// @tparam C Type of std::vector that is accessed.
//////////////////////////////////////////////////////////////////////
template <typename C>
struct stl_vector_accessor
  : public accessor_base<typename C::value_type, stl_vector_accessor<C>>
{
private:
  using index_type = typename container_traits<C>::domain_type::index_type;

  C*          m_container;
  index_type m_index;

  typedef typename C::value_type value_type;

  friend class accessor_core_access;

public:
  stl_vector_accessor(C* container, index_type index)
    : m_container(container),
      m_index(index)
  { }

  bool is_local(void) const
  {
    return true;
  }

  bool is_null(void) const
  {
    return m_container == NULL;
  }

  template<typename F>
  void apply_set(const F& f) const
  {
    f(m_container->operator[](m_index));
  }

  template<typename F>
  typename F::result_type apply_get(F const& f) const
  {
    return f(m_container->operator[](m_index));
  }

  void define_type(typer& t)
  {
    t.member(m_container);
    t.member(m_index);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Defines a const_accessor for std::vector.
///
/// @tparam C Type of std::vector that is accessed.
//////////////////////////////////////////////////////////////////////
template <typename C>
struct stl_vector_const_accessor
  : public accessor_base<typename C::value_type, stl_vector_const_accessor<C>>
{
private:
  using index_type = typename container_traits<C>::domain_type::index_type;

  C const*    m_container;
  index_type  m_index;

  typedef typename C::value_type value_type;

  friend class accessor_core_access;

public:

  stl_vector_const_accessor(C const* container, index_type index)
    : m_container(container),
      m_index(index)
  { }

  bool is_local(void) const
  {
    return true;
  }

  bool is_null(void) const
  {
    return m_container == NULL;
  }

  template<typename F>
  typename F::result_type apply_get(F const& f) const
  {
    return f(m_container->operator[](m_index));
  }

  void define_type(typer& t)
  {
    t.member(m_container);
    t.member(m_index);
  }
};

template<typename C>
struct accessor_traits<stl_vector_accessor<C>>
{
  using is_localized = std::true_type;
};

template<typename C>
struct accessor_traits<stl_vector_const_accessor<C>>
{
  using is_localized = std::true_type;
};

} // namespace stapl

#endif /* STAPL_VIEWS_PROXY_STL_VECTOR_ACCESSOR_HPP */
