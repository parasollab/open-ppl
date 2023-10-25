/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEW_HANDLE_ACCESSOR_HPP
#define STAPL_VIEW_HANDLE_ACCESSOR_HPP

#include <stapl/runtime.hpp>
#include <stapl/utility/invoke_arg.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Defines an accessor over a p_object, keeping a rmi_handle
///        reference to perform rmi communications.
/// @tparam T Type of object (derived) from @ref p_object.
/// @todo Use proper broadcast primitives.
//////////////////////////////////////////////////////////////////////
template <typename T>
struct handle_accessor
{
private:
  typedef rmi_handle::reference         ref_t;

  T*                                    m_t_ptr;
  /* const */ ref_t                     m_handle_ref;

public:
  typedef T value_type;

  handle_accessor(T* t_ptr)
    : m_t_ptr(t_ptr),
      m_handle_ref(t_ptr->get_rmi_handle())
  { }

  handle_accessor(ref_t const& ref)
    : m_t_ptr(NULL),
      m_handle_ref(ref)
  { }

  void define_type(typer&)
  {
    stapl_assert(0, "handle_accessor isn't packable at the moment");
  }

  template<typename Class, typename... Args>
  void invoke(void (Class::* const memberFuncPtr)(Args...),
              typename invoke_arg<Args>::type const&... args)

  {
    if (m_t_ptr)
      (m_t_ptr/*.get()*/->*memberFuncPtr)(args...);
    else
      async_rmi(all_locations, m_handle_ref, memberFuncPtr, args...);
  }

  template<typename Class, typename Rtn, typename... Args>
  Rtn invoke(Rtn (Class::* const memberFuncPtr)(Args...),
             typename invoke_arg<Args>::type const&... args)
  {
    if (m_t_ptr)
      return (m_t_ptr->*memberFuncPtr)(args...);
    else
      return restore(m_handle_ref, memberFuncPtr, args...).get();
  }

  template<typename Class, typename Rtn, typename... Args>
  Rtn const_invoke(Rtn (Class::* const memberFuncPtr)(Args...) const,
                   typename invoke_arg<Args>::type const&... args)
  {
    if (m_t_ptr)
      return (m_t_ptr->*memberFuncPtr)(args...);
    else
      return restore(m_handle_ref, memberFuncPtr, args...).get();
  }
}; // class handle_accessor

} // namespace stapl

#endif // ifndef STAPL_VIEW_HANDLE_ACCESSOR_HPP
