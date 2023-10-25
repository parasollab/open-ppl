/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_IMMUTABLE_SHARED_HPP
#define STAPL_RUNTIME_IMMUTABLE_SHARED_HPP

#include "serialization.hpp"
#include "type_traits/is_basic.hpp"
#include "type_traits/is_copyable.hpp"
#include <memory>
#include <tuple>
#include <type_traits>
#include <utility>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Describes an immutable object.
///
/// @tparam T Object type.
///
/// Immutable objects may be passed by reference when communication happens in
/// shared memory. The object can never be mutated and it is deleted when the
/// last @ref immutable_shared is destroyed.
///
/// Locations that are on shared memory may have a single object accessible
/// through their @ref immutable_shared. Locations that are on different address
/// spaces are guaranteed to have distinct copies of the object.
///
/// @see immutable_wrapper
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename T,
         bool = (sizeof(T)<=sizeof(std::shared_ptr<T>) && is_basic<T>::value)>
class immutable_shared
{
public:
  typedef T element_type;

private:
  std::shared_ptr<const T> m_p;

public:
  template<typename... Args>
  explicit immutable_shared(Args&&... args)
  : m_p(std::make_shared<const T>(std::forward<Args>(args)...))
  { }

  immutable_shared(immutable_shared const&) = default;
  immutable_shared(immutable_shared&&) = default;

  operator T const&(void) const noexcept
  { return *m_p; }

  T const& get(void) const noexcept
  { return *m_p; }

  long use_count(void) const
  { return m_p.use_count(); }

  bool unique(void) const
  { return m_p.unique(); }

  void define_type(typer& t)
  { t.member(m_p); }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref immutable_shared for basic types for which
///        <tt>sizeof(T)<=sizeof(std::shared_ptr<T>)</tt>.
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename T>
class immutable_shared<T, true>
{
public:
  typedef const T       element_type;
  typedef std::tuple<T> member_types;

private:
  T m_t;

public:
  template<typename... Args>
  explicit immutable_shared(Args&&... args)
  : m_t(std::forward<Args>(args)...)
  { }

  immutable_shared(immutable_shared const&) = default;
  immutable_shared(immutable_shared&&) = default;

  operator T const&(void) const noexcept
  { return m_t; }

  T const& get(void) const noexcept
  { return m_t; }

  constexpr long use_count(void) const noexcept
  { return 1l; }

  constexpr bool unique(void) const noexcept
  { return true; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Constructs an immutable object of type @p T.
///
/// Immutable objects may be passed by reference when communication happens in
/// shared memory.
///
/// The object can never be mutated and it is deleted when the last
/// @ref immutable_shared is destroyed.
///
/// @see immutable
/// @related immutable_shared
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename T, typename... Args>
immutable_shared<T> make_immutable_shared(Args&&... args)
{
  return immutable_shared<T>(std::forward<Args>(args)...);
}


namespace runtime {

////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_copyable for @ref immutable_shared.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_copyable<immutable_shared<T, false>>
: public std::true_type
{ };

} // namespace runtime

} // namespace stapl

#endif
