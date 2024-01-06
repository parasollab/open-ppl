/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_TYPE_TRAITS_LAZY_STORAGE_HPP
#define STAPL_RUNTIME_TYPE_TRAITS_LAZY_STORAGE_HPP

#include <type_traits>
#include <utility>

namespace stapl {

////////////////////////////////////////////////////////////////////
/// @brief Provides lazy construction of objects with automatic storage.
///
/// @tparam T     Object type.
/// @tparam Align Object alignment.
///
/// This struct is a POD and while it reserves stack space for the object, the
/// object is initialized only when it is explicitly required to.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T, std::size_t Align = std::alignment_of<T>::value>
struct lazy_storage
{
private:
  ////////////////////////////////////////////////////////////////////
  /// @brief Calls the destructor of the stored object.
  ////////////////////////////////////////////////////////////////////
  struct destructor
  {
    T& m_t;

    constexpr explicit destructor(T& t) noexcept
    : m_t(t)
    { }

    ~destructor(void)
    { m_t.~T(); }
  };

  typename std::aligned_storage<sizeof(T), Align>::type m_storage;

public:
  template<typename... U>
  void construct(U&&... u)
  { new(&m_storage) T(std::forward<U>(u)...); }

  void destroy(void) noexcept
  { reinterpret_cast<T*>(&m_storage)->~T(); }

  T const& get(void) const noexcept
  { return *reinterpret_cast<const T*>(&m_storage); }

  T& get(void) noexcept
  { return *reinterpret_cast<T*>(&m_storage); }

  T moveout(void)
  {
    destructor d{get()};
    return std::move(get());
  }
};

} // namespace stapl

#endif
