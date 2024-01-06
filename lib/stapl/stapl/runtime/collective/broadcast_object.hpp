/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_COLLECTIVE_BROADCAST_OBJECT_HPP
#define STAPL_RUNTIME_COLLECTIVE_BROADCAST_OBJECT_HPP

#include "../context.hpp"
#include "../rmi_handle.hpp"
#include "../value_handle.hpp"
#include "../non_rmi/response.hpp"

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Performs a broadcast over all locations of the current gang.
///
/// @tparam T Object type.
///
/// @ingroup runtimeCollectives
///
/// @todo Use platform optimized broadcast implementation.
//////////////////////////////////////////////////////////////////////
template<typename T>
class broadcast_object
: public value_handle<T>
{
public:
  using value_type    = T;
private:
  using response_type = handle_response<packed_handle_type, broadcast_object>;

  friend response_type;

  rmi_handle m_handle;

public:
  explicit broadcast_object(context& ctx)
  : m_handle(ctx, this)
  { }

  rmi_handle::reference const& get_rmi_handle(void) noexcept
  { return m_handle; }

  rmi_handle::const_reference const& get_rmi_handle(void) const noexcept
  { return m_handle; }

private:
  using value_handle<T>::set_value;

public:
  void operator()(T const& t)
  { response_type{}(m_handle, t); }

  void operator()(T&& t)
  { response_type{}(m_handle, std::move(t)); }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref broadcast_object for @c void.
///
/// @ingroup runtimeCollectives
//////////////////////////////////////////////////////////////////////
template<>
class broadcast_object<void>
: public value_handle<void>
{
public:
  using value_type    = void;
private:
  using response_type = handle_response<packed_handle_type, broadcast_object>;

  friend response_type;

  rmi_handle m_handle;

public:
  explicit broadcast_object(context& ctx)
  : m_handle(ctx, this)
  { }

  rmi_handle::reference const& get_rmi_handle(void) noexcept
  { return m_handle; }

  rmi_handle::const_reference const& get_rmi_handle(void) const noexcept
  { return m_handle; }

private:
  using value_handle<void>::set_value;

public:
  void operator()(void)
  { response_type{}(m_handle); }
};

} // namespace runtime

} // namespace stapl

#endif
