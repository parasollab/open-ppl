/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_INSTRUMENTATION_NO_COMM_GUARD_HPP
#define STAPL_RUNTIME_INSTRUMENTATION_NO_COMM_GUARD_HPP

#include "../exception.hpp"
#include "../primitive_traits.hpp"
#include "../this_context.hpp"
#include <mutex>
#include <unordered_map>
#include <utility>

namespace stapl {

////////////////////////////////////////////////////////////////////
/// @brief Registers the given context as a no communication section of code.
///
/// If any communication happens within the scope of a @ref no_comm_guard
/// object, then the execution aborts with an error message.
///
/// @ref no_comm_guard is used during development to guarantee that a piece of
/// code never does any kind of unexpected communication.
///
/// @ingroup instrumentation
////////////////////////////////////////////////////////////////////
class no_comm_guard
{
public:
  typedef runtime::context_id                        key_type;
private:
  typedef std::unordered_map<key_type, unsigned int> container_type;

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns the container of registered objects and the container's
  ///        mutex.
  ////////////////////////////////////////////////////////////////////
  static std::pair<container_type, std::mutex>& get_container(void)
  {
    static std::pair<container_type, std::mutex> map;
    return map;
  }

  const key_type m_key;

public:
  ////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if a @ref no_comm_guard is enabled for the given
  ///        context id.
  ////////////////////////////////////////////////////////////////////
  static bool enabled(key_type const& key)
  {
    auto& r = get_container();
    auto& c = r.first;

    std::lock_guard<std::mutex> lock{r.second};
    return (c.find(key)!=c.end());
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Enables the no-communication checking for the current context.
  ////////////////////////////////////////////////////////////////////
  no_comm_guard(void)
  : m_key(runtime::this_context::get_id())
  {
    auto& r = get_container();
    auto& c = r.first;

    std::lock_guard<std::mutex> lock{r.second};
    ++(c[m_key]);
  }

  no_comm_guard(no_comm_guard const&) = delete;
  no_comm_guard& operator=(no_comm_guard const&) = delete;

  ////////////////////////////////////////////////////////////////////
  /// @brief Disables the no-communication checking for the current context.
  ////////////////////////////////////////////////////////////////////
  ~no_comm_guard(void)
  {
    auto& r = get_container();
    auto& c = r.first;

    std::lock_guard<std::mutex> lock{r.second};
    auto it = c.find(m_key);
    STAPL_RUNTIME_ASSERT(it!=c.end() && (it->second)>0);
    if (--(it->second)==0)
      c.erase(it);
  }
};

} // namespace stapl


////////////////////////////////////////////////////////////////////
/// @brief Detects if a @ref stapl::no_comm_guard is enabled and a section of
///        code does communication.
///
/// @ingroup instrumentation
////////////////////////////////////////////////////////////////////
# define STAPL_RUNTIME_CALL_NO_COMM_GUARD(traits)                            \
{                                                                            \
  if ((traits & stapl::runtime::primitive_traits::comm) &&                   \
      stapl::no_comm_guard::enabled(stapl::runtime::this_context::get_id())) \
    stapl::runtime::warning(                                                 \
      "STAPL warning: Communication while stapl::no_comm_guard active",      \
      BOOST_CURRENT_FUNCTION);                                               \
}

#endif
