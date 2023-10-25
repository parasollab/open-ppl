/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_PROMISE_HPP
#define STAPL_RUNTIME_PROMISE_HPP

#include "aggregator.hpp"
#include "future.hpp"
#include "runqueue.hpp"
#include "value_handle.hpp"
#include "non_rmi/response.hpp"
#include <tuple>
#include <utility>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Provides a means of setting a result asynchronously, which may be
///        retrieved through an instance of @ref future.
///
/// @tparam R Returned object type.
///
/// @see future
/// @ingroup ARMIUtilities
///
/// @todo Type required in member types to avoid packing the pointer.
/// @todo Delete copy constructor.
/// @todo Handle the case of creating a promise and not using it.
/// @todo Handle the case of calling @ref get_future() multiple times.
/// @todo Handle broken promises.
//////////////////////////////////////////////////////////////////////
template<typename R>
class promise
{
public:
  typedef std::tuple</* T*, */process_id> member_types;
private:
  typedef runtime::value_handle<R>        handle_type;

  handle_type* m_handle;
  process_id   m_pid;

public:
  promise(void)
  : m_handle(new handle_type),
    m_pid(runtime::runqueue::get_process_id())
  { }

#if 0
  // see @todo for copy constructor
  promise(promise const&) = delete;
#else
  promise(promise const&) noexcept = default;
#endif
  promise& operator=(promise const&) = delete;

  promise(promise&& other) noexcept
  : m_handle(other.m_handle),
    m_pid(other.m_pid)
  {
    other.m_handle = nullptr;
    other.m_pid    = invalid_process_id;
  }

  promise& operator=(promise&& other) noexcept
  {
    delete m_handle;
    m_handle       = other.m_handle;
    m_pid          = other.m_pid;
    other.m_handle = nullptr;
    other.m_pid    = invalid_process_id;
    return *this;
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns a @ref future associated with the promised result.
  ////////////////////////////////////////////////////////////////////
  future<R> get_future(void)
  { return future<R>{std::unique_ptr<handle_type>{m_handle}}; }

  ////////////////////////////////////////////////////////////////////
  /// @brief Stores the value and makes the promise ready.
  ////////////////////////////////////////////////////////////////////
  void set_value(R const& value)
  {
    typedef runtime::response<handle_type> response_type;
    response_type r{*m_handle};
    r(m_pid, value);
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Stores the value and makes the promise ready.
  ////////////////////////////////////////////////////////////////////
  void set_value(R&& value)
  {
    typedef runtime::response<handle_type> response_type;
    response_type r{*m_handle};
    r(m_pid, std::move(value));
  }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref promise for @c void.
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<>
class promise<void>
{
public:
  typedef std::tuple</* T*, */process_id> member_types;
private:
  typedef runtime::value_handle<void>     handle_type;

  handle_type* m_handle;
  process_id   m_pid;

public:
  promise(void)
  : m_handle(new handle_type),
    m_pid(runtime::runqueue::get_process_id())
  { }

#if 0
  promise(promise const&) = delete;
#else
  promise(promise const&) noexcept = default;
#endif
  promise& operator=(promise const&) = delete;

  promise(promise&& other) noexcept
  : m_handle(other.m_handle),
    m_pid(other.m_pid)
  {
    other.m_handle = nullptr;
    other.m_pid    = invalid_process_id;
  }

  promise& operator=(promise&& other) noexcept
  {
    delete m_handle;
    m_handle       = other.m_handle;
    m_pid          = other.m_pid;
    other.m_handle = nullptr;
    other.m_pid    = invalid_process_id;
    return *this;
  }

  future<void> get_future(void)
  { return future<void>{std::unique_ptr<handle_type>{m_handle}}; }

  void set_value(void)
  {
    typedef runtime::response<handle_type> response_type;
    response_type r{*m_handle};
    r(m_pid);
  }
};

} // namespace stapl

#endif
