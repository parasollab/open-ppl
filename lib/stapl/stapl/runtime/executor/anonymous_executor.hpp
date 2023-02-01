/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_RUNTIME_EXECUTOR_ANONYMOUS_EXECUTOR_HPP
#define STAPL_RUNTIME_EXECUTOR_ANONYMOUS_EXECUTOR_HPP

#include "../rmi_handle.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Provides support for getting a @ref p_object on the gang of
///        @c stapl_main().
///
/// @ingroup executors
///
/// @deprecated The PARAGRAPH's use of this for out of group communication
///   has been replaced by executor_rmi.  It is used in a few places to serve
///   as a well known p_object (i.e., @ref stapl::serial_io), but this
///   usage needs to served elsewhere and this class removed.
//////////////////////////////////////////////////////////////////////
class anonymous_executor
{
private:
  rmi_handle m_handle;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref anonymous_executor and pushes it on the stack.
  //////////////////////////////////////////////////////////////////////
  anonymous_executor(void);

  //////////////////////////////////////////////////////////////////////
  /// @brief Pops the @ref anonymous_executor from the stack and destroys it.
  //////////////////////////////////////////////////////////////////////
  ~anonymous_executor(void);

  anonymous_executor(anonymous_executor const&) = delete;
  anonymous_executor& operator=(anonymous_executor const&) = delete;

  rmi_handle::const_reference const& get_rmi_handle(void) const noexcept
  { return m_handle; }

  rmi_handle::reference const& get_rmi_handle(void) noexcept
  { return m_handle; }

  unsigned int get_location_id(void) noexcept
  { return m_handle.get_location_id(); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Returns the instance of the @ref anonymous_executor.
///
/// @ingroup executors
//////////////////////////////////////////////////////////////////////
anonymous_executor& get_anonymous_executor(void);

} // namespace stapl

#endif
