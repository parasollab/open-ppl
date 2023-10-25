/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_INSTRUMENTATION_CALLBACK_HPP
#define STAPL_RUNTIME_INSTRUMENTATION_CALLBACK_HPP

#include "../exception.hpp"
#include "../this_context.hpp"
#include <algorithm>
#include <functional>
#include <list>
#include <mutex>
#include <unordered_map>
#include <utility>

namespace stapl {

////////////////////////////////////////////////////////////////////
/// @brief Registers the given function as a callback to be called when a
///        primitive is called in the registering context.
///
/// The registered functions have to have the following signature
/// @code
/// void f(const char* s);
/// @endcode
/// where @p s is the name of the primitive that called it.
///
/// @ingroup instrumentation
////////////////////////////////////////////////////////////////////
class callback
{
public:
  typedef runtime::full_location                  key_type;
private:
  typedef std::function<void(const char*)>        function_type;
  typedef std::list<function_type>                list_type;
  typedef std::unordered_map<key_type, list_type> container_type;

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns the container of registered objects and the container's
  ///        mutex.
  ////////////////////////////////////////////////////////////////////
  static std::pair<container_type, std::mutex>& get_container(void)
  {
    static std::pair<container_type, std::mutex> map;
    return map;
  }

  const key_type      m_key;
  list_type::iterator m_it;

public:
  ////////////////////////////////////////////////////////////////////
  /// @brief Call all registered callback functions for the given context id
  ///        and passes the given string.
  ////////////////////////////////////////////////////////////////////
  static void call(runtime::context_id const& id, const char* s)
  {
    const key_type key = id.current;
    auto& r            = get_container();
    auto& c            = r.first;

    std::lock_guard<std::mutex> lock{r.second};
    auto it = c.find(key);
    if (it!=c.end()) {
      for (function_type& f : it->second)
        f(s);
    }
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref callback object that is registered in the
  ///        current context.
  ////////////////////////////////////////////////////////////////////
  template<typename Function>
  explicit callback(Function&& f)
  : m_key(runtime::this_context::get_id().current)
  {
    auto& r = get_container();
    auto& c = r.first;

    std::lock_guard<std::mutex> lock{r.second};
    auto& l = c[m_key];
    m_it = l.emplace_front(std::forward<Function>(f));
  }

  callback(callback const&) = delete;
  callback& operator=(callback const&) = delete;

  ~callback(void)
  {
    auto& r = get_container();
    auto& c = r.first;

    std::lock_guard<std::mutex> lock{r.second};
    auto it = c.find(m_key);
    STAPL_RUNTIME_ASSERT(it!=c.end());
    auto& l = it->second;
    l.erase(m_it);
    if (l.empty())
      c.erase(it);
  }
};


////////////////////////////////////////////////////////////////////
/// @brief Calls the registered @ref stapl::callback objects in the current
///        context with the given arguments.
///
/// @ingroup instrumentation
////////////////////////////////////////////////////////////////////
#define STAPL_RUNTIME_CALL_CALLBACKS(s)                                   \
{                                                                         \
  stapl::runtime::context* ctx = stapl::runtime::this_context::try_get(); \
  if (ctx)                                                                \
    stapl::callback::call(ctx->get_id(), s);                              \
}

} // namespace stapl

#endif
