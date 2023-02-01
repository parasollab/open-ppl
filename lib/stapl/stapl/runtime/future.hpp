/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_FUTURE_HPP
#define STAPL_RUNTIME_FUTURE_HPP

#include "context.hpp"
#include "exception.hpp"
#include "instrumentation.hpp"
#include "message.hpp"
#include "runqueue.hpp"
#include "request/rmi_request.hpp"
#include <algorithm>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <type_traits>
#include <utility>
#include <vector>

namespace stapl {

template<typename T>
class future;

template<typename T>
class futures;


namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Calls function @p f with a @p Future created with @p t....
///
/// @see future_base, futures_base
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename Function, typename Future>
class continuation_request final
: public rmi_request
{
private:
  Function m_f;
  Future   m_future;

public:
  static constexpr std::size_t expected_size(void) noexcept
  { return sizeof(continuation_request); }

  template<typename F, typename... T>
  explicit continuation_request(F&& f, T&&... t)
  : rmi_request(sizeof(*this)),
    m_f(std::forward<F>(f)),
    m_future(std::forward<T>(t)...)
  { }

  bool operator()(context&) final
  {
    m_f(std::move(m_future));
    this->~continuation_request();
    return true;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Provides a common base for an asynchronously returned object.
///
/// @tparam T Object type.
///
/// Continuations created through @ref async_then() execute in a new context
/// in the next nesting level than the context they are created in. When reused,
/// the context of the continuation remains the same between invocations.
///
/// @see future
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename T>
class future_base
{
public:
  using value_type    = T;
private:
  using future_type   = future<T>;
  using function_type = std::function<void(future_type)>;

  function_type                     m_fun;
  boost::intrusive_ptr<location_md> m_loc;
  context::id                       m_cid;
  context::epoch_type               m_epoch;
protected:
  mutable std::mutex                m_mtx;
private:
  bool                              m_owned;

public:
  future_base(void)
  : m_epoch(logical_clock::no_time),
    m_owned(false)
  { }

  virtual ~future_base(void) = default;

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Sets up a continuation.
  //////////////////////////////////////////////////////////////////////
  template<typename Function>
  void setup_continuation(Function&& f)
  {
    m_fun = std::forward<Function>(f);

    if (!m_loc) {
      // setup metadata for continuation invocation
      auto& ctx = this_context::get();
      if (ctx.get_nesting()==std::numeric_limits<nesting_level>::max())
        STAPL_RUNTIME_ERROR("Too many recursive calls to async_then().");
      m_loc        = &(ctx.get_location_md());
      m_cid        = ctx.get_id();
      ++m_cid.nesting;
      m_epoch      = ctx.get_epoch();
    }

    // increase pending requests to satisfy the fence
    m_loc->get_fence_md().add_pending();
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if the result has been received.
  //////////////////////////////////////////////////////////////////////
  virtual bool valid_no_yield(void) const noexcept = 0;

  //////////////////////////////////////////////////////////////////////
  /// @brief Schedules a continuation if @ref async_then() was called.
  //////////////////////////////////////////////////////////////////////
  void schedule_continuation(void)
  {
    if (!m_fun)
      return;

    using request_type = continuation_request<function_type, future_type>;
    const std::size_t size = request_type::expected_size();
    auto m = message::create(header::RMI,
                             size,
                             header::request{m_cid, m_epoch});
    new(m->reserve(size)) request_type{std::move(m_fun), *this};
    runqueue::add(m_loc->get_gang_md(), m_loc->get_id(), true, std::move(m));
  }

public:
  void set_owned(void) noexcept
  { m_owned = true; }

  bool is_owned(void) const noexcept
  { return m_owned; }

  virtual T get(void) = 0;

  virtual bool valid(void) const = 0;

  virtual void wait(void) const = 0;

  template<typename Function>
  void async_then(Function&& f)
  {
    std::unique_lock<std::mutex> lock{m_mtx};

    STAPL_RUNTIME_ASSERT(!bool(m_fun));

    if (valid_no_yield()) {
      // result received, call the continuation directly
      lock.unlock();
      f(future_type{*this});
      return;
    }

    // result not received yet, schedule_continuation() will be called from the
    // thread that will set the result
    setup_continuation(std::forward<Function>(f));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Provides a common base for asynchronously returned arrays of objects.
///
/// @tparam T Object type.
///
/// Continuations created through @ref async_then() execute in a new context
/// in the next nesting level than the context they are created in. When reused,
/// the context of the continuation remains the same between invocations.
///
/// @see futures
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename T>
class futures_base
{
public:
  using value_type            = T;
  using size_type             = std::size_t;
  using aggregate_result_type = typename std::conditional<
                                  std::is_void<T>::value,
                                  void,
                                  std::vector<T>>::type;
private:
  using future_type           = futures<T>;
  using function_type         = std::function<void(future_type)>;

  function_type                     m_fun;
  boost::intrusive_ptr<location_md> m_loc;
  context::id                       m_cid;
  context::epoch_type               m_epoch;
protected:
  mutable std::mutex                m_mtx;
private:
  bool                              m_owned;

public:
  futures_base(void)
  : m_epoch(logical_clock::no_time),
    m_owned(false)
  { }

  virtual ~futures_base(void) = default;

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Sets up a continuation.
  //////////////////////////////////////////////////////////////////////
  template<typename Function>
  void setup_continuation(Function&& f)
  {
    m_fun = std::forward<Function>(f);

    if (!m_loc) {
      // setup metadata for continuation invocation
      auto& ctx = this_context::get();
      if (ctx.get_nesting()==std::numeric_limits<nesting_level>::max())
        STAPL_RUNTIME_ERROR("Too many recursive calls to async_then().");
      m_loc        = &(ctx.get_location_md());
      m_cid        = ctx.get_id();
      ++m_cid.nesting;
      m_epoch      = ctx.get_epoch();
    }

    // increase pending requests to satisfy the fence
    m_loc->get_fence_md().add_pending();
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if the result has been received.
  //////////////////////////////////////////////////////////////////////
  virtual bool valid_no_yield(void) const noexcept = 0;

  //////////////////////////////////////////////////////////////////////
  /// @brief Schedules a continuation if @ref async_then() was called.
  //////////////////////////////////////////////////////////////////////
  void schedule_continuation(void)
  {
    if (!m_fun)
      return;

    using request_type = continuation_request<function_type, future_type>;
    const std::size_t size = request_type::expected_size();
    auto m = message::create(header::RMI,
                             size,
                             header::request{m_cid, m_epoch});
    new(m->reserve(size)) request_type{std::move(m_fun), *this};
    runqueue::add(m_loc->get_gang_md(), m_loc->get_id(), true, std::move(m));
  }

public:
  void set_owned(void) noexcept
  { m_owned = true; }

  bool is_owned(void) const noexcept
  { return m_owned; }

  virtual size_type size(void) const = 0;

  virtual T get(const size_type) = 0;

  virtual aggregate_result_type get(void) = 0;

  virtual bool valid(const size_type) const = 0;

  virtual bool valid(void) const = 0;

  virtual void wait(const size_type) const  = 0;

  virtual void wait(void) const = 0;

  template<typename Function>
  void async_then(Function&& f)
  {
    std::unique_lock<std::mutex> lock{m_mtx};

    STAPL_RUNTIME_ASSERT(!bool(m_fun));

    if (valid_no_yield()) {
      // all results received, call the continuation directly
      lock.unlock();
      f(future_type{*this});
      return;
    }

    // all results not received yet, schedule_continuation() will be called from
    // the thread that will set the last result
    setup_continuation(std::forward<Function>(f));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief @ref future_base implementation for ready values.
///
/// @tparam T Object type.
///
/// @see future
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename T>
class ready_future final
: public future_base<T>
{
private:
  T m_t;

public:
  explicit ready_future(T const& t)
  : m_t(t)
  { }

  explicit ready_future(T&& t)
  : m_t(std::move(t))
  { }

protected:
  bool valid_no_yield(void) const noexcept final
  { return true; }

public:
  bool valid(void) const final
  { return true; }

  void wait(void) const final
  { }

  T get(void) final
  { return std::move(m_t); }
};

//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref ready_future for @c void.
///
/// @see future
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<>
class ready_future<void> final
: public future_base<void>
{
protected:
  bool valid_no_yield(void) const noexcept final
  { return true; }

public:
  bool valid(void) const final
  { return true; }

  void wait(void) const final
  { }

  void get(void) final
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief @ref futures_base implementation for ready values.
///
/// @tparam T Object type.
///
/// @see futures
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename T>
class ready_futures final
: public futures_base<T>
{
public:
  using size_type = typename futures_base<T>::size_type;

private:
  std::vector<T> m_storage;

public:
  template<typename InputIterator>
  ready_futures(InputIterator first, InputIterator last)
  : m_storage(first, last)
  { }

protected:
  bool valid_no_yield(void) const noexcept final
  { return true; }

public:
  size_type size(void) const noexcept final
  { return m_storage.size(); }

  bool valid(const size_type) const final
  { return true; }

  bool valid(void) const final
  { return true; }

  void wait(const size_type) const final
  { }

  void wait(void) const final
  { }

  T get(const size_type n) final
  { return std::move(m_storage[n]); }

  std::vector<T> get(void) final
  { return std::move(m_storage); }
};

//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref ready_futures for @c void.
///
/// @see futures
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<>
class ready_futures<void> final
: public futures_base<void>
{
public:
  using size_type = typename futures_base<void>::size_type;

private:
  size_type m_size;

public:
  explicit ready_futures(const size_type n)
  : m_size(n)
  { }

protected:
  bool valid_no_yield(void) const noexcept final
  { return true; }

public:
  size_type size(void) const noexcept final
  { return m_size; }

  bool valid(const size_type) const final
  { return true; }

  bool valid(void) const final
  { return true; }

  void wait(const size_type) const final
  { }

  void wait(void) const final
  { }

  void get(const size_type) final
  { }

  void get(void) final
  { }
};

} // namespace runtime


//////////////////////////////////////////////////////////////////////
/// @brief Provides a mechanism to access the result of asynchronous operations.
///
/// @tparam T Object type.
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename T>
class future
{
public:
  using handle_type = typename runtime::future_base<T>;

private:
  std::unique_ptr<handle_type> m_handle_ptr;

public:
  future(void) = default;

  explicit future(std::unique_ptr<handle_type> p)
  : m_handle_ptr(std::move(p))
  { m_handle_ptr->set_owned(); }

  explicit future(handle_type& h)
  : m_handle_ptr(&h)
  { }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns the result.
  ///
  /// This function waits until the result is available and returns it.
  /// @c valid()==false after a call to this method.
  ////////////////////////////////////////////////////////////////////
  T get(void)
  {
    using runtime::primitive_traits;
    STAPL_RUNTIME_PROFILE("future::get()", (primitive_traits::blocking |
                                            primitive_traits::yield));
    if (!m_handle_ptr)
      STAPL_RUNTIME_ERROR("Inexistant shared state.");
    if (!m_handle_ptr->is_owned())
      return m_handle_ptr.release()->get();
    return std::unique_ptr<handle_type>{std::move(m_handle_ptr)}->get();
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if the result is available.
  ////////////////////////////////////////////////////////////////////
  bool valid(void) const
  {
    using runtime::primitive_traits;
    STAPL_RUNTIME_PROFILE("future::valid()", (primitive_traits::non_blocking |
                                              primitive_traits::yield));
    return (!m_handle_ptr ? false : m_handle_ptr->valid());
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Waits for the result to become available.
  ///
  /// Blocks until @c valid()==true.
  ////////////////////////////////////////////////////////////////////
  void wait(void) const
  {
    using runtime::primitive_traits;
    STAPL_RUNTIME_PROFILE("future::wait()", (primitive_traits::blocking |
                                             primitive_traits::yield));
    if (!m_handle_ptr)
      STAPL_RUNTIME_ERROR("Inexistant shared state.");
    m_handle_ptr->wait();
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Executes @c f when the result is available.
  ///
  /// If the result is already available, then the function is executed
  /// in-place.
  ////////////////////////////////////////////////////////////////////
  template<typename Function>
  void async_then(Function&& f)
  {
    using runtime::primitive_traits;
    STAPL_RUNTIME_PROFILE("future::async_then()",
                          primitive_traits::non_blocking);
    if (!m_handle_ptr)
      STAPL_RUNTIME_ERROR("Inexistant shared state.");
    m_handle_ptr.release()->async_then(std::forward<Function>(f));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Returns a @ref future with the given value in it.
///
/// @related ready_future
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template <typename T>
future<typename std::decay<T>::type> make_ready_future(T&& value)
{
  using value_type  = typename std::decay<T>::type;
  using future_type = runtime::ready_future<value_type>;
  return future<value_type>{
           std::unique_ptr<future_type>{new future_type{std::forward<T>(value)}}
         };
}

//////////////////////////////////////////////////////////////////////
/// @brief Returns a @c future<void> that is ready.
///
/// @related ready_future
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
inline future<void> make_ready_future(void)
{
  using future_type = runtime::ready_future<void>;
  return future<void>{std::unique_ptr<future_type>{new future_type}};
}


//////////////////////////////////////////////////////////////////////
/// @brief Provides a mechanism to access the result of asynchronous operations.
///
/// @tparam T Object type.
///
/// The result in this case is one or more objects, so there are functions to
/// check, wait on and return individual elements.
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename T>
class futures
{
public:
  using handle_type           = runtime::futures_base<T>;
  using size_type             = typename handle_type::size_type;
  using aggregate_result_type = typename handle_type::aggregate_result_type;

private:
  std::unique_ptr<handle_type> m_handle_ptr;
  bool                         m_consumed_individual;

public:
  futures(void)
  : m_consumed_individual(false)
  { }

  explicit futures(std::unique_ptr<handle_type> p)
  : m_handle_ptr(std::move(p)),
    m_consumed_individual(false)
  { m_handle_ptr->set_owned(); }

  explicit futures(handle_type& h)
  : m_handle_ptr(&h),
    m_consumed_individual(false)
  { }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns how many results are or will be stored.
  ////////////////////////////////////////////////////////////////////
  size_type size(void) const
  { return m_handle_ptr->size(); }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns the n-th result.
  ///
  /// This function waits until the n-th result is available and returns it.
  /// @c valid(n)==false after a call to this method.
  ///
  /// @warning You cannot mix calls of this function and @ref get().
  ////////////////////////////////////////////////////////////////////
  T get(const size_type n)
  {
    using runtime::primitive_traits;
    STAPL_RUNTIME_PROFILE("futures::get()", (primitive_traits::blocking |
                                             primitive_traits::yield));
    if (!m_handle_ptr)
      STAPL_RUNTIME_ERROR("Inexistant shared state.");
    m_consumed_individual = true;
    return m_handle_ptr->get(n);
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns all the results.
  ///
  /// This function waits until all results are available and returns them.
  /// @c valid()==false after a call to this method.
  ///
  /// @warning You cannot mix calls of this function and
  ///          @ref get(const size_type).
  ////////////////////////////////////////////////////////////////////
  aggregate_result_type get(void)
  {
    using runtime::primitive_traits;
    STAPL_RUNTIME_PROFILE("futures::get()", (primitive_traits::blocking |
                                             primitive_traits::yield));
    if (!m_handle_ptr)
      STAPL_RUNTIME_ERROR("Inexistant shared state.");
    if (m_consumed_individual)
      STAPL_RUNTIME_ERROR("futures::get(n) was called.");
    if (!m_handle_ptr->is_owned())
      return m_handle_ptr.release()->get();
    return std::unique_ptr<handle_type>{std::move(m_handle_ptr)}->get();
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if the n-th result is available.
  ////////////////////////////////////////////////////////////////////
  bool valid(const size_type n) const
  {
    using runtime::primitive_traits;
    STAPL_RUNTIME_PROFILE("futures::valid()", (primitive_traits::non_blocking |
                                               primitive_traits::yield));
    return (!m_handle_ptr ? false : m_handle_ptr->valid(n));
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if all the results are available.
  ////////////////////////////////////////////////////////////////////
  bool valid(void) const
  {
    using runtime::primitive_traits;
    STAPL_RUNTIME_PROFILE("futures::valid()", (primitive_traits::non_blocking |
                                               primitive_traits::yield));
    return (!m_handle_ptr ? false : m_handle_ptr->valid());
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Waits for the n-th result to become available.
  ///
  /// Blocks until @c valid(n)==true.
  ////////////////////////////////////////////////////////////////////
  void wait(const size_type n) const
  {
    using runtime::primitive_traits;
    STAPL_RUNTIME_PROFILE("futures::wait()", (primitive_traits::blocking |
                                              primitive_traits::yield));
    if (!m_handle_ptr)
      STAPL_RUNTIME_ERROR("Inexistant shared state.");
    m_handle_ptr->wait(n);
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Waits for all the results to become available.
  ///
  /// Blocks until @c valid()==true.
  ////////////////////////////////////////////////////////////////////
  void wait(void) const
  {
    using runtime::primitive_traits;
    STAPL_RUNTIME_PROFILE("futures::wait()", (primitive_traits::blocking |
                                              primitive_traits::yield));
    if (!m_handle_ptr)
      STAPL_RUNTIME_ERROR("Inexistant shared state.");
    if (m_consumed_individual)
      STAPL_RUNTIME_ERROR("futures::get(n) was called.");
    m_handle_ptr->wait();
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Executes @c f when the results are available.
  ///
  /// If the result is already available, then the function is executed
  /// in-place.
  ////////////////////////////////////////////////////////////////////
  template<typename Function>
  void async_then(Function&& f)
  {
    using runtime::primitive_traits;
    STAPL_RUNTIME_PROFILE("futures::async_then()",
                          primitive_traits::non_blocking);
    if (!m_handle_ptr)
      STAPL_RUNTIME_ERROR("Inexistant shared state.");
    if (m_consumed_individual)
      STAPL_RUNTIME_ERROR("futures::get(n) was called.");
    m_handle_ptr.release()->async_then(std::forward<Function>(f));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Returns a @ref futures with the given values in it.
///
/// @related ready_futures
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename InputIterator>
auto make_ready_futures(InputIterator first, InputIterator last)
  -> futures<typename std::decay<decltype(*first)>::type>
{
  using value_type  = typename std::decay<decltype(*first)>::type;
  using future_type = runtime::ready_futures<value_type>;
  return futures<value_type>{
           std::unique_ptr<future_type>{new future_type{first, last}}
         };
}

//////////////////////////////////////////////////////////////////////
/// @brief Returns a @c futures<void> that is ready.
///
/// @related ready_futures
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename Size>
futures<void> make_ready_futures(Size n)
{
  using future_type = runtime::ready_futures<void>;
  return futures<void>{std::unique_ptr<future_type>{new future_type{n}}};
}

} // namespace stapl

#endif
