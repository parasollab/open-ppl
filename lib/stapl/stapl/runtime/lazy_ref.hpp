/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_LAZY_REF_HPP
#define STAPL_RUNTIME_LAZY_REF_HPP

#include "rmi_handle.hpp"
#include "serialization_fwd.hpp"
#include "type_traits/is_p_object.hpp"
#include <functional>
#include <memory>
#include <type_traits>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief @c std::reference_wrapper -like class for distributed objects.
///
/// This reference wrapper will only try to retrieve the distributed object
/// when the @ref lazy_reference_wrapper::get() or the implicit conversion
/// operator to @p T are called.
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename T>
class lazy_reference_wrapper
{
public:
  using type = T;
private:
  using non_cv_type = typename std::remove_cv<T>::type;

  rmi_handle::light_reference m_handle;

public:
  lazy_reference_wrapper(T& t) noexcept
  : m_handle(const_cast<non_cv_type&>(t).get_rmi_handle())
  { }

  lazy_reference_wrapper(T&&) = delete;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a reference to the object.
  /// @todo Decide whether lazy_ref should be used inter-gang.  If not,
  ///  remove resolve_handle usage here and directly use in place where
  ///  lazy_ref is called in inter-gang contexts.
  //////////////////////////////////////////////////////////////////////
  T& get(void) const noexcept
  {
    T* const t = resolve_handle<non_cv_type>(m_handle);
    STAPL_RUNTIME_ASSERT(t);
    return *t;
  }

  operator T&(void) const noexcept
  { return get(); }

  void define_type(typer& t)
  { t.member(m_handle); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Returns a pointer to the distributed object referenced by @p r.
///
/// This function is required for interoperability with @c boost::bind().
///
/// @related lazy_reference_wrapper
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename T>
T* get_pointer(lazy_reference_wrapper<T> const& r)
{
  return std::addressof(r.get());
}


//////////////////////////////////////////////////////////////////////
/// @brief Helper functions that creates an object of type
///        @ref lazy_reference_wrapper if @p T is a distributed object,
///        otherwise an object of type @c std::reference_wrapper.
///
/// @related lazy_reference_wrapper
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename T>
typename std::conditional<
           is_p_object<T>::value,
           lazy_reference_wrapper<T>,
           std::reference_wrapper<T>
         >::type lazy_ref(T& t)
{
  return t;
}


//////////////////////////////////////////////////////////////////////
/// @brief Helper function that creates an object of type
///        @ref lazy_reference_wrapper if @p T is a distributed object,
///        otherwise an object of type @c std::reference_wrapper.
///
/// @related lazy_reference_wrapper
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename T>
typename std::conditional<
           is_p_object<T>::value,
           lazy_reference_wrapper<const T>,
           std::reference_wrapper<const T>
         >::type lazy_cref(T const& t)
{
  return t;
}

} // namespace stapl

#endif
