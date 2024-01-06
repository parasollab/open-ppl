/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_POINTER_HPP
#define STAPL_RUNTIME_POINTER_HPP

#include "rmi_handle.hpp"
#include "serialization_fwd.hpp"
#include "type_traits/is_p_object.hpp"
#include <cstddef>
#include <functional>
#include <memory>
#include <type_traits>

namespace stapl {

template<typename T, bool>
class pointer_wrapper;

//////////////////////////////////////////////////////////////////////
/// @brief @ref pointer_wrapper when it is known that @p T is not a distributed
///        object.
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename T>
using object_pointer_wrapper = pointer_wrapper<T, false>;

//////////////////////////////////////////////////////////////////////
/// @brief @ref pointer_wrapper when it is known that @p T is a distributed
///        object.
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename T>
using p_object_pointer_wrapper = pointer_wrapper<T, true>;


//////////////////////////////////////////////////////////////////////
/// @brief @c std::reference_wrapper -like class for pointers to objects.
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename T, bool = is_p_object<T>::value>
class pointer_wrapper
{
public:
  using type = T;

private:
  T* m_t;

  template<typename, bool>
  friend class pointer_wrapper;

public:
  constexpr pointer_wrapper(void) noexcept
  : m_t(nullptr)
  { }

  template<typename U>
  constexpr pointer_wrapper(U* ptr) noexcept
  : m_t(ptr)
  { }

  constexpr pointer_wrapper(std::nullptr_t) noexcept
  : m_t(nullptr)
  { }

  constexpr pointer_wrapper(pointer_wrapper const&) noexcept = default;

  template<typename U>
  constexpr pointer_wrapper(object_pointer_wrapper<U> const& other) noexcept
  : m_t(other.m_t)
  { }

  template<typename U>
  constexpr pointer_wrapper(p_object_pointer_wrapper<U> const& other) noexcept
  : m_t(other.m_t)
  { }

  constexpr pointer_wrapper(pointer_wrapper&&) noexcept = default;

  template<typename U>
  pointer_wrapper(object_pointer_wrapper<U>&& other) noexcept
  : m_t(other.m_t)
  {
    other = nullptr;
  }

  template<typename U>
  pointer_wrapper(p_object_pointer_wrapper<U>&& other) noexcept
  : m_t(other.m_t)
  {
    other = nullptr;
  }

  template<typename U>
  pointer_wrapper& operator=(U* ptr) noexcept
  {
    m_t = ptr;
    return *this;
  }

  pointer_wrapper& operator=(std::nullptr_t) noexcept
  {
    m_t = nullptr;
    return *this;
  }

  pointer_wrapper& operator=(pointer_wrapper const&) noexcept = default;

  template<typename U>
  pointer_wrapper& operator=(object_pointer_wrapper<U> const& other) noexcept
  {
    m_t = other.m_t;
    return *this;
  }

  template<typename U>
  pointer_wrapper& operator=(p_object_pointer_wrapper<U> const& other) noexcept
  {
    m_t = other.m_t;
    return *this;
  }

  pointer_wrapper& operator=(pointer_wrapper&&) noexcept = default;

  template<typename U>
  pointer_wrapper& operator=(object_pointer_wrapper<U>&& other) noexcept
  {
    m_t   = other.m_t;
    other = nullptr;
    return *this;
  }

  template<typename U>
  pointer_wrapper& operator=(p_object_pointer_wrapper<U>&& other) noexcept
  {
    m_t   = other.m_t;
    other = nullptr;
    return *this;
  }

  void swap(pointer_wrapper& other) noexcept
  { std::swap(m_t, other.m_t); }

  T* get(void) const noexcept
  { return m_t; }

  operator T*(void) const noexcept
  { return m_t; }

  T& operator*(void) const noexcept
  { return *m_t; }

  T* operator->(void) const noexcept
  { return m_t; }

  explicit operator bool(void) const noexcept
  { return m_t; }

  void define_type(typer& t)
  { t.member(m_t); }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref pointer_wrapper for pointers to distributed
///        objects.
///
/// This pointer wrapper will attempt to retrieve the distributed object at
/// unpacking and cache it for subsequent calls.
///
/// It allows access to distributed objects that are in a different gang,
/// provided that they exist in the same affinity where the @ref pointer_wrapper
/// object is unpacked. Otherwise, it holds @c nullptr.
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename T>
class pointer_wrapper<T, true>
{
public:
  using type = T;

private:
  using non_cv_type = typename std::remove_cv<T>::type;

  rmi_handle::reference m_h;
  T*                    m_t;

  template<typename, bool>
  friend class pointer_wrapper;

public:
  pointer_wrapper(void) noexcept
  : m_t(nullptr)
  { }

  template<typename U>
  pointer_wrapper(U* ptr) noexcept
  : m_t(ptr)
  {
    if (m_t)
      m_h = const_cast<non_cv_type*>(m_t)->get_rmi_handle();
  }

  pointer_wrapper(std::nullptr_t) noexcept
  : m_t(nullptr)
  { }

  pointer_wrapper(pointer_wrapper const&) noexcept = default;

  template<typename U>
  pointer_wrapper(object_pointer_wrapper<U> const& other) noexcept
  : m_t(other.m_t)
  {
    if (m_t)
      m_h = const_cast<non_cv_type*>(m_t)->get_rmi_handle();
  }

  template<typename U>
  pointer_wrapper(p_object_pointer_wrapper<U> const& other) noexcept
  : m_h(other.m_h),
    m_t(other.m_t)
  { }

  pointer_wrapper(pointer_wrapper&&) noexcept = default;

  template<typename U>
  pointer_wrapper(object_pointer_wrapper<U>&& other) noexcept
  : m_t(other.m_t)
  {
    if (m_t) {
      other = nullptr;
      m_h   = const_cast<non_cv_type*>(m_t)->get_rmi_handle();
    }
  }

  template<typename U>
  pointer_wrapper(p_object_pointer_wrapper<U>&& other) noexcept
  : m_h(other.m_h),
    m_t(other.m_t)
  {
    other = nullptr;
  }

  template<typename U>
  pointer_wrapper& operator=(U* ptr) noexcept
  {
    m_t = ptr;
    if (m_t)
      m_h = const_cast<non_cv_type*>(m_t)->get_rmi_handle();
    return *this;
  }

  pointer_wrapper& operator=(std::nullptr_t) noexcept
  {
    m_h = rmi_handle::reference{};
    m_t = nullptr;
    return *this;
  }

  pointer_wrapper& operator=(pointer_wrapper const&) noexcept = default;

  template<typename U>
  pointer_wrapper& operator=(object_pointer_wrapper<U> const& other) noexcept
  {
    m_t = other.m_t;
    if (m_t)
      m_h = const_cast<non_cv_type*>(m_t)->get_rmi_handle();
  }

  template<typename U>
  pointer_wrapper& operator=(p_object_pointer_wrapper<U> const& other) noexcept
  {
    m_h = other.m_h;
    m_t = other.m_t;
    return *this;
  }

  pointer_wrapper& operator=(pointer_wrapper&&) noexcept = default;

  template<typename U>
  pointer_wrapper& operator=(object_pointer_wrapper<U>&& other) noexcept
  {
    m_t = other.m_t;
    if (m_t) {
      other = nullptr;
      m_h   = const_cast<non_cv_type*>(m_t)->get_rmi_handle();
    }
  }

  template<typename U>
  pointer_wrapper& operator=(p_object_pointer_wrapper<U>&& other) noexcept
  {
    m_h   = other.m_h;
    m_t   = other.m_t;
    other = nullptr;
    return *this;
  }

  void swap(pointer_wrapper& other) noexcept
  {
    std::swap(m_h, other.m_h);
    std::swap(m_t, other.m_t);
  }

  T* get(void) const noexcept
  { return m_t; }

  operator T*(void) const noexcept
  { return m_t; }

  T& operator*(void) const noexcept
  { return *m_t; }

  T* operator->(void) const noexcept
  { return m_t; }

  explicit operator bool(void) const noexcept
  { return m_t; }

  rmi_handle::reference const& handle(void) const noexcept
  { return m_h; }

  void define_type(typer& t)
  {
    t.member(m_h);
    t.transient(m_t, resolve_handle<non_cv_type>(m_h));
  }
};


template<typename T, typename U>
bool operator==(pointer_wrapper<T> const& x, pointer_wrapper<U> const& y)
{ return (x.get()==y.get()); }

template<typename T, typename U>
bool operator!=(pointer_wrapper<T> const& x, pointer_wrapper<U> const& y)
{ return (x.get()!=y.get()); }

template<typename T, typename U>
bool operator<(pointer_wrapper<T> const& x, pointer_wrapper<U> const& y)
{ return (x.get()<y.get()); }

template<typename T, typename U>
bool operator>(pointer_wrapper<T> const& x, pointer_wrapper<U> const& y)
{ return (x.get()>y.get()); }

template<typename T, typename U>
bool operator<=(pointer_wrapper<T> const& x, pointer_wrapper<U> const& y)
{ return (x.get()<=y.get()); }

template<typename T, typename U>
bool operator>=(pointer_wrapper<T> const& x, pointer_wrapper<U> const& y)
{ return (x.get()>=y.get()); }


//////////////////////////////////////////////////////////////////////
/// @brief Returns a pointer to the object through @ref pointer_wrapper.
///
/// This function is required for interoperability with @c boost::bind().
///
/// @related pointer_wrapper
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename T>
T* get_pointer(pointer_wrapper<T> const& r)
{
  return r.get();
}


//////////////////////////////////////////////////////////////////////
/// @brief Helper functions that creates an object of type @ref pointer_wrapper.
///
/// @related pointer_wrapper
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename T>
pointer_wrapper<T> pointer(T* const t)
{
  return t;
}


//////////////////////////////////////////////////////////////////////
/// @brief Helper function that creates an object of type @ref pointer_wrapper.
///
/// @related pointer_wrapper
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename T>
pointer_wrapper<const T> cpointer(T* const t)
{
  return t;
}

} // namespace stapl

#endif
