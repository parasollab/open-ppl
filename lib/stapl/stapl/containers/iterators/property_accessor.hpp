/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_PROPERTY_ACCESSOR_HPP
#define STAPL_CONTAINERS_PROPERTY_ACCESSOR_HPP

#include <stapl/containers/type_traits/container_traits.hpp>
#include <stapl/containers/graph/functional.hpp>

#include <stapl/views/proxy/proxy.hpp>
#include <stapl/views/proxy/accessor_base.hpp>
#include <stapl/utility/invoke_arg.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Accessor for the property on vertices of a graph.
/// @tparam Container Type of the pGraph container.
//////////////////////////////////////////////////////////////////////
template <typename Container>
class property_accessor
{
public:
  using container_type = Container;
  using index_type     = typename container_traits<Container>::gid_type;
  using value_type     =
    typename container_traits<Container>::value_type::property_type;

protected:

  template <typename Derived, typename A, typename C, typename D>
  friend class iterator_facade;

  friend class accessor_core_access;

  container_type*  m_container;
  index_type       m_index;

public:
  bool is_null() const
  {
    return m_container->domain().empty();
  };

  void define_type(typer& t)
  {
    t.member(m_container);
    t.member(m_index);
  }

  property_accessor(void) = default;
  property_accessor(property_accessor const&) = default;

  property_accessor(null_reference const&)
  { }

  property_accessor(container_type* container, index_type const& index)
    : m_container(container), m_index(index)
  { }

  value_type read() const
  {
    return m_container->get_element(m_index).property();
  }

  value_type ref() const
  {
    return this->read();
  }

  template<typename T>
  void write(T&& val) const
  {
    m_container->vp_apply_async(m_index,
      stapl::detail::assign_val<value_type>(std::forward<T>(val)));
  }

  template<typename F>
  void apply_set(F const& f) const
  {
    m_container->vp_apply_async(m_index, f);
  }

  template<typename F>
  typename F::result_type apply_get(F const& f) const
  {
    return m_container->vp_apply(m_index, f);
  }

  template<typename Class, typename... Args>
  void invoke(void (Class::* const memberFuncPtr)(Args...),
              typename invoke_arg<Args>::type const&... args) const

  {
      m_container->vp_apply_async(m_index,
        detail::apply_set_helper<
          void (Class::*)(Args...), typename std::decay<Args>::type...
        >(memberFuncPtr, args...));
  }

  template<typename Class, typename Rtn, typename... Args>
  Rtn invoke(Rtn (Class::* const memberFuncPtr)(Args...),
             typename invoke_arg<Args>::type const&... args) const
  {
    return m_container->vp_apply(m_index,
      detail::apply_get_helper<
        Rtn (Class::*)(Args...), typename std::decay<Args>::type...
      >(memberFuncPtr, args...));
  }

  template<typename Class, typename Rtn, typename... Args>
  Rtn const_invoke(Rtn (Class::* const memberFuncPtr)(Args...) const,
                   typename invoke_arg<Args>::type const&... args) const

  {
    return m_container->vp_apply(m_index,
      detail::const_apply_get_helper<
        Rtn (Class::*)(Args...) const, typename std::decay<Args>::type...
      > (memberFuncPtr, args...));
  }
}; // class property_accessor

} // namespace stapl

#endif // STAPL_CONTAINERS_PROPERTY_ACCESSOR_HPP
