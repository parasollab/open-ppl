/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_IMMUTABLE_REF_HPP
#define STAPL_RUNTIME_IMMUTABLE_REF_HPP

#include "serialization.hpp"
#include "type_traits/is_basic.hpp"
#include "type_traits/is_copyable.hpp"
#include <type_traits>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Wraps a reference to a temporarily immutable object.
///
/// @tparam T Object type.
///
/// Referents of immutable references may avoid serialization when communication
/// happens in shared memory. The referent must not be deleted or mutated until
/// all @ref immutable_reference_wrapper to it have been deleted. This is
/// commonly guaranteed with synchronization.
///
/// Once all the @ref immutable_reference_wrapper objects have been destroyed,
/// then the referenced object can be mutated or deleted.
///
/// @see immutable_shared
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename T, bool = (sizeof(T)<=sizeof(T*) && is_basic<T>::value)>
class immutable_reference_wrapper
{
public:
  using type = T;

private:
  T const* m_t;

public:
  immutable_reference_wrapper(T const& t) noexcept
  : m_t(std::addressof(t))
  { }

  immutable_reference_wrapper(T&&) = delete;

  immutable_reference_wrapper(immutable_reference_wrapper const&) noexcept =
    default;
  immutable_reference_wrapper(immutable_reference_wrapper&&) noexcept = default;

  operator T const&(void) const noexcept
  { return *m_t; }

  T const& get(void) const noexcept
  { return *m_t; }

  void define_type(typer& t)
  { t.member(m_t); }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref immutable_reference_wrapper for basic types
///        for which <tt>sizeof(T)<=sizeof(T*)</tt>.
///
/// @see is_basic
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename T>
class immutable_reference_wrapper<T, true>
{
public:
  using type = T;

private:
  T m_t;

public:
  immutable_reference_wrapper(T const& t) noexcept
  : m_t(t)
  { }

  immutable_reference_wrapper(T&&) = delete;

  immutable_reference_wrapper(immutable_reference_wrapper const&) noexcept =
    default;
  immutable_reference_wrapper(immutable_reference_wrapper&&) noexcept = default;

  operator T const&(void) const noexcept
  { return m_t; }

  T const& get(void) const noexcept
  { return m_t; }

  void define_type(typer& t)
  { t.member(m_t); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Creates an immutable reference to @p t.
///
/// An immutable reference may pass the object by reference during shared memory
/// communication, avoiding any potential copies.
///
/// @warning The sender has to guarantee that all callees have finished before
///          deleting or mutating the object.
///
/// @see make_immutable_shared
/// @related immutable_reference_wrapper
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename T>
immutable_reference_wrapper<T> immutable_ref(T const& t)
{
  return immutable_reference_wrapper<T>{t};
}


namespace runtime {

////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_copyable for
///        @ref immutable_reference_wrapper.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_copyable<immutable_reference_wrapper<T>>
: public std::true_type
{ };

} // namespace runtime

} // namespace stapl

#endif
