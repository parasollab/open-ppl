/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_COLLECTIVE_ALLGATHER_OBJECT_HPP
#define STAPL_RUNTIME_COLLECTIVE_ALLGATHER_OBJECT_HPP

#include "../aggregator.hpp"
#include "../context.hpp"
#include "../rmi_handle.hpp"
#include "../value_handle.hpp"
#include <utility>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Performs an allgather over all locations of the current gang.
///
/// @tparam T Object type.
///
/// @ingroup runtimeCollectives
///
/// @todo Use platform optimized allgather implementation
///       (e.g., MPI_Iallgather).
//////////////////////////////////////////////////////////////////////
template<typename T>
class allgather_object
: public values_handle<T>
{
public:
  using value_type    = T;
private:
  using response_type =
    indexed_handle_response<packed_handle_type, allgather_object>;

  friend response_type;

  rmi_handle m_handle;

public:
  explicit allgather_object(context& ctx)
  : values_handle<T>(ctx.get_gang_md().size()),
    m_handle(ctx, this)
  { }

  rmi_handle::reference const& get_rmi_handle(void) noexcept
  { return m_handle; }

  rmi_handle::const_reference const& get_rmi_handle(void) const noexcept
  { return m_handle; }

private:
  using values_handle<T>::set_value;

public:
  void operator()(T const& t)
  { response_type{}(m_handle, t); }

  void operator()(T&& t)
  { response_type{}(m_handle, std::move(t)); }
};

} // namespace runtime

} // namespace stapl

#endif
