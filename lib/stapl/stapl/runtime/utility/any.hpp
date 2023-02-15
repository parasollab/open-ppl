/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_UTILITY_ANY_HPP
#define STAPL_RUNTIME_UTILITY_ANY_HPP

#include "../exception.hpp"
#include <algorithm>
#include <typeinfo>
#include <utility>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief @ref any can hold an object of any type. The object can be retrieved
///        only by casting to the correct type.
///
/// Provides the functionality of @c boost::any without dynamic_casts or
/// checks when in non-debug mode.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
class any
{
private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Base class for holding any object.
  //////////////////////////////////////////////////////////////////////
  class placeholder
  {
  public:
    placeholder(void) = default;
    placeholder(placeholder const&) = delete;
    placeholder& operator=(placeholder const&) = delete;
    virtual ~placeholder(void) = default;
    virtual std::type_info const& type(void) const noexcept = 0;
    virtual placeholder* clone(void) const = 0;
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Implementation of @ref placeholder for objects of type @p T.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  class holder final
  : public placeholder
  {
  private:
    T m_t;

  public:
    explicit holder(T const& t)
    : m_t(t)
    { }

    explicit holder(T&& t)
    : m_t(std::move(t))
    { }

    std::type_info const& type(void) const noexcept
    { return typeid(T); }

    placeholder* clone(void) const
    { return new holder(m_t); }

    T const& get(void) const noexcept
    { return m_t; }

    T& get(void) noexcept
    { return m_t; }
  };

  placeholder* m_p;

public:
  constexpr any(void) noexcept
  : m_p(nullptr)
  { }

  template<typename T>
  any(T&& t)
  : m_p(new holder<typename std::decay<T>::type>(std::forward<T>(t)))
  { }

  any(any&& other)
  : m_p(other.m_p)
  { other.m_p = nullptr; }

  any(any const& other)
  : m_p(other.empty() ? nullptr : other.m_p->clone())
  { }

  ~any(void)
  { delete m_p; }

  template<typename T>
  any& operator=(T&& other)
  {
    any(std::forward<T>(other)).swap(*this);
    return *this;
  }

  any& operator=(any other)
  {
    other.swap(*this);
    return *this;
  }

  any& swap(any& other)
  {
    using std::swap;
    swap(m_p, other.m_p);
    return *this;
  }

  bool empty(void) const noexcept
  { return !m_p; }

  std::type_info const& type(void) const noexcept
  { return (empty() ? typeid(void) : m_p->type()); }

  template<typename T>
  T const* try_get(void) const noexcept
  {
    if (empty())
      return nullptr;
    STAPL_RUNTIME_ASSERT(typeid(T)==type());
    return &(static_cast<holder<T>*>(m_p)->get());
  }

  template<typename T>
  T* try_get(void) noexcept
  {
    if (empty())
      return nullptr;
    STAPL_RUNTIME_ASSERT(typeid(T)==type());
    return &(static_cast<holder<T>*>(m_p)->get());
  }

  template<typename T>
  T const& get(void) const noexcept
  { return *try_get<T>(); }

  template<typename T>
  T& get(void) noexcept
  { return *try_get<T>(); }
};

} // namespace runtime

} // namespace stapl

#endif
