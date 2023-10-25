/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_PROXY_ACCESSOR_BASE_HPP
#define STAPL_VIEWS_PROXY_ACCESSOR_BASE_HPP

#include <stapl/views/proxy/accessor.hpp>
#include <stapl/views/proxy/accessor_traits.hpp>
#include <stapl/views/proxy/as_reference.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/tuple.hpp>
#include <stapl/runtime/type_traits/callable_traits.hpp>

namespace stapl {

template <typename T>
struct identity;

template<typename T>
class container_wrapper_ref;

template<typename T>
T& unwrap_container_wrapper(T& t);

template<typename T>
T& unwrap_container_wrapper(container_wrapper_ref<T> t);


namespace detail {

template<typename U>
class assign_ref
{
private:
  U m_x;

public:
  assign_ref(U const& u)
    : m_x(u)
  { }

  using result_type = void;

  template<typename T>
  void operator()(T& y) const
  {
    y = std::move(m_x);
  }

  void define_type(typer& t)
  {
    t.member(m_x);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor which stores a pointer to member function and
/// corresponding arguments, making the invocation on an object passed
/// to it when called.  Handles void returning invocations for
/// @ref accessor_base, avoiding compilation overhead of lambda + bind
/// operation.
//////////////////////////////////////////////////////////////////////
template<typename PMF, typename... Args>
class apply_set_helper
{
private:
  PMF                 m_pmf;
  std::tuple<Args...> m_args;

public:
  apply_set_helper(PMF const& pmf, Args const&... args)
    : m_pmf(pmf), m_args(args...)
  { }

  using result_type = void;

  template<typename T>
  void operator()(T& t) const
  {
    tuple_ops::apply(unwrap_container_wrapper(t), m_pmf, m_args);
  }

  template<typename T, typename Accessor>
  result_type operator()(proxy<T, Accessor> const& t) const
  {
    auto& as_ref = as_reference(t);
    return tuple_ops::apply(unwrap_container_wrapper(as_ref), m_pmf, m_args);
  }

  void define_type(typer& t)
  {
    t.member(m_pmf);
    t.member(m_args);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor which stores a pointer to member function and
/// corresponding arguments, making the invocation on an object passed
/// to it when called.  Handles non-void returning, mutating invocations for
/// @ref accessor_base, avoiding compilation overhead of lambda + bind
/// operation.
//////////////////////////////////////////////////////////////////////
template<typename PMF, typename... Args>
class apply_get_helper
{
private:
  PMF                 m_pmf;
  std::tuple<Args...> m_args;

public:
  apply_get_helper(PMF pmf, Args const&... args)
    : m_pmf(pmf), m_args(args...)
  { }

  using result_type = typename callable_traits<PMF>::result_type;

  template<typename T>
  result_type operator()(T& t) const
  {
    return tuple_ops::apply(unwrap_container_wrapper(t), m_pmf, m_args);
  }

  template<typename T, typename Accessor>
  result_type operator()(proxy<T, Accessor> const& t) const
  {
    auto& as_ref = as_reference(t);
    return tuple_ops::apply(unwrap_container_wrapper(as_ref), m_pmf, m_args);
  }

  void define_type(typer& t)
  {
    t.member(m_pmf);
    t.member(m_args);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor which stores a pointer to member function and
/// corresponding arguments, making the invocation on an object passed
/// to it when called.  Handles non-void returning, non-mutating invocations for
/// @ref accessor_base, avoiding compilation overhead of lambda + bind
/// operation.
//////////////////////////////////////////////////////////////////////
template<typename PMF, typename... Args>
class const_apply_get_helper
{
private:
  PMF                 m_pmf;
  std::tuple<Args...> m_args;

public:
  const_apply_get_helper(PMF pmf, Args const&... args)
    : m_pmf(pmf), m_args(args...)
  { }

  using result_type = typename callable_traits<PMF>::result_type;

  template<typename T>
  result_type operator()(T const& t) const
  {
    return tuple_ops::apply(unwrap_container_wrapper(t), m_pmf, m_args);
  }

  template<typename T, typename Accessor>
  result_type operator()(proxy<T, Accessor> const& t) const
  {
    auto& as_ref = as_reference(t);
    return tuple_ops::apply(unwrap_container_wrapper(as_ref), m_pmf, m_args);
  }

  void define_type(typer& t)
  {
    t.member(m_pmf);
    t.member(m_args);
  }
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Accessor base class. Provides the basic operation for read
///        and write values.
/// @tparam T Value type.
/// @tparam Derived Type of the most derived accessor.
//////////////////////////////////////////////////////////////////////
template<typename T, typename Derived>
class accessor_base
{
  Derived const& derived(void) const
  {
    return static_cast<Derived const&>(*this);
  }

  using value_type = T;

public:
  value_type read(void) const
  {
    return derived().apply_get(stapl::identity<value_type>());
  }

  template<typename U>
  void write(U const& value) const
  {
    derived().apply_set(detail::assign_ref<U>(value));
  }

  template<typename Class, typename... Args>
  void invoke(void (Class::* const memberFuncPtr)(Args...),
              typename std::decay<Args>::type const&... args) const

  {
    derived().apply_set(detail::apply_set_helper<
                          void (Class::*)(Args...),
                          typename std::decay<Args>::type...
                        >(memberFuncPtr, args...));
  }

  template<typename Class, typename Rtn, typename... Args>
  Rtn invoke(Rtn (Class::* const memberFuncPtr)(Args...),
             typename std::decay<Args>::type const&... args) const
  {
    return derived().apply_get(detail::apply_get_helper<
                                 Rtn (Class::*)(Args...),
                                 typename std::decay<Args>::type...
                               >(memberFuncPtr, args...));
  }

  template<typename Class, typename Rtn, typename... Args>
  Rtn const_invoke(Rtn (Class::* const memberFuncPtr)(Args...) const,
                   typename std::decay<Args>::type const&... args) const

  {
    return derived().apply_get(detail::const_apply_get_helper<
                                 Rtn (Class::*)(Args...) const,
                                 typename std::decay<Args>::type...
                               >(memberFuncPtr, args...));
  }
}; // class accessor_base

} // namespace stapl

#endif // ifndef STAPL_VIEWS_PROXY_ACCESSOR_BASE_HPP

