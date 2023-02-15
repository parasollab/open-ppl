/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_VALUE_HANDLE_HPP
#define STAPL_RUNTIME_VALUE_HANDLE_HPP

#include "async_result.hpp"
#include "future.hpp"
#include "yield.hpp"
#include <utility>
#include <vector>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Handle for receiving one result.
///
/// @tparam R Result type.
///
/// @see async_result, future, future_base
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename R>
class value_handle
: public future_base<R>
{
private:
  typedef async_result<R>                              internal_storage_type;
public:
  typedef typename internal_storage_type::storage_type storage_type;

private:
  internal_storage_type m_storage;

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if the result has been received.
  //////////////////////////////////////////////////////////////////////
  bool valid_no_yield(void) const noexcept final
  { return m_storage.valid(); }

public:
  ////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if the result has been received.
  ////////////////////////////////////////////////////////////////////
  bool valid(void) const final
  { return yield_if_not([this] { return this->m_storage.valid(); }); }

  ////////////////////////////////////////////////////////////////////
  /// @brief Waits for the result to be received.
  ///
  /// Blocks until @ref valid() const returns @c true.
  ////////////////////////////////////////////////////////////////////
  void wait(void) const final
  { yield_until([this] { return this->m_storage.valid(); }); }

  ////////////////////////////////////////////////////////////////////
  /// @copydoc wait() const
  ////////////////////////////////////////////////////////////////////
  void wait(context& ctx) const
  { yield_until(ctx, [this] { return this->m_storage.valid(); }); }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns the received result.
  ///
  /// This function waits until the result has been received and returns it.
  ///
  /// Calling @ref valid() const returns @c false after a call to this function.
  ///
  /// The lock ensures that @ref set_value() has finished before the @ref get()
  /// continues, to avoid calling @ref schedule_continuation() on an object that
  /// has been destroyed.
  ////////////////////////////////////////////////////////////////////
  R get(void) final
  {
    wait();
    std::lock_guard<std::mutex> lock{this->m_mtx};
    return m_storage.get();
  }

  ////////////////////////////////////////////////////////////////////
  /// @copydoc get()
  ////////////////////////////////////////////////////////////////////
  R get(context& ctx)
  {
    wait(ctx);
    std::lock_guard<std::mutex> lock{this->m_mtx};
    return m_storage.get();
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Sets the result.
  ///
  /// This function keeps a pointer to the buffer where the result remains
  /// packed until @ref get() is called.
  ///
  /// @param p    Pointer to the @ref stapl::runtime::arg_storage in the buffer.
  /// @param base Address in the buffer that the
  ///             @ref stapl::runtime::arg_storage is stored in.
  /// @param m    Buffer that contains the @c stapl::runtime::arg_storage.
  ///
  /// @see arg_storage, message
  ////////////////////////////////////////////////////////////////////
  void set_value(storage_type* const p, void* const base, message_shared_ptr& m)
  {
    std::lock_guard<std::mutex> lock{this->m_mtx};
    m_storage.set_value(p, base, m);
    this->schedule_continuation();
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Sets the result.
  ///
  /// @param value Value to be set.
  ////////////////////////////////////////////////////////////////////
  void set_value(R const& value)
  {
    std::lock_guard<std::mutex> lock{this->m_mtx};
    m_storage.set_value(value);
    this->schedule_continuation();
  }

  ////////////////////////////////////////////////////////////////////
  /// @copydoc set_value(R const&)
  ////////////////////////////////////////////////////////////////////
  void set_value(R&& value)
  {
    std::lock_guard<std::mutex> lock{this->m_mtx};
    m_storage.set_value(std::move(value));
    this->schedule_continuation();
  }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref value_handle for @c void.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<>
class value_handle<void>
: public future_base<void>
{
private:
  typedef async_result<void> internal_storage_type;

  internal_storage_type m_storage;

protected:
  bool valid_no_yield(void) const noexcept final
  { return m_storage.valid(); }

public:
  bool valid(void) const final
  { return yield_if_not([this] { return this->m_storage.valid(); }); }

  void wait(void) const final
  { yield_until([this] { return this->m_storage.valid(); }); }

  void wait(context& ctx) const
  { yield_until(ctx, [this] { return this->m_storage.valid(); }); }

  void get(void) final
  {
    wait();
    std::lock_guard<std::mutex> lock{this->m_mtx};
    m_storage.get();
  }

  void get(context& ctx)
  {
    wait(ctx);
    std::lock_guard<std::mutex> lock{this->m_mtx};
    m_storage.get();
  }

  void set_value(void)
  {
    std::lock_guard<std::mutex> lock{this->m_mtx};
    m_storage.set_value();
    this->schedule_continuation();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Handle for receiving multiple results.
///
/// @tparam R Result type.
///
/// @see async_results, futures, futures_base
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename R>
class values_handle
: public futures_base<R>
{
private:
  typedef async_results<R>                             internal_storage_type;
public:
  typedef typename futures_base<R>::size_type          size_type;
  typedef typename internal_storage_type::storage_type storage_type;

private:
  internal_storage_type m_storage;

public:
  explicit values_handle(const size_type n) noexcept
  : m_storage(n)
  { }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if the result has been received.
  //////////////////////////////////////////////////////////////////////
  bool valid_no_yield(void) const noexcept final
  { return m_storage.valid(); }

public:
  size_type size(void) const noexcept final
  { return m_storage.size(); }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if the n-th result has been received.
  ////////////////////////////////////////////////////////////////////
  bool valid(const size_type n) const final
  { return yield_if_not([this, n] { return this->m_storage.valid(n); }); }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if all the results have been received.
  ////////////////////////////////////////////////////////////////////
  bool valid(void) const final
  { return yield_if_not([this] { return this->m_storage.valid(); }); }

  ////////////////////////////////////////////////////////////////////
  /// @brief Waits for the n-th result to be received.
  ///
  /// Blocks until @ref valid(const size_type) const returns @c true.
  ////////////////////////////////////////////////////////////////////
  void wait(const size_type n) const final
  { yield_until([this, n] { return this->m_storage.valid(n); }); }

  ////////////////////////////////////////////////////////////////////
  /// @brief Waits for all the results to be received.
  ///
  /// Blocks until @ref valid() const returns @c true.
  ////////////////////////////////////////////////////////////////////
  void wait(void) const final
  { yield_until([this] { return this->m_storage.valid(); }); }

  ////////////////////////////////////////////////////////////////////
  /// @copydoc wait()
  ////////////////////////////////////////////////////////////////////
  void wait(context& ctx) const
  { yield_until(ctx, [this] { return this->m_storage.valid(); }); }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns the n-th received result.
  ///
  /// This function waits until the n-th result has been received and returns
  /// it.
  ///
  /// Calling @ref valid() const or @ref valid(const size_type) const returns
  /// @c false after a call to this function.
  ///
  /// The lock ensures that @ref set_value() has finished before the @ref get()
  /// continues, to avoid calling @ref schedule_continuation() on an object that
  /// has been destroyed.
  ///
  /// @warning You cannot mix calls of this function and @c get().
  ////////////////////////////////////////////////////////////////////
  R get(const size_type n) final
  {
    wait(n);
    std::lock_guard<std::mutex> lock{this->m_mtx};
    return m_storage.get(n);
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns all the received results.
  ///
  /// This function waits until all results have been received and returns
  /// them.
  ///
  /// Calling @ref valid() const or @ref valid(const size_type) const returns
  /// @c false after a call to this function.
  ///
  /// The lock ensures that @ref set_value() has finished before the @ref get()
  /// continues, to avoid calling @ref schedule_continuation() on an object that
  /// has been destroyed.
  ///
  /// @warning You cannot mix calls of this function and @ref get(size_type).
  ////////////////////////////////////////////////////////////////////
  std::vector<R> get(void) final
  {
    wait();
    std::lock_guard<std::mutex> lock{this->m_mtx};
    return m_storage.get();
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Sets the n-th result.
  ///
  /// This function keeps a pointer to the buffer where the result remains
  /// packed until @ref get() or @ref get(const size_type) are called.
  ///
  /// @param n    Result index.
  /// @param p    Pointer to the @c stapl::runtime::arg_storage in the buffer.
  /// @param base Address in the buffer that the
  ///             @ref stapl::runtime::arg_storage is stored in.
  /// @param m    Buffer that contains the @ref stapl::runtime::arg_storage.
  ///
  /// @see arg_storage, message
  ////////////////////////////////////////////////////////////////////
  void set_value(const size_type n,
                 storage_type* const p, void* const base, message_shared_ptr& m)
  {
    std::lock_guard<std::mutex> lock{this->m_mtx};
    const bool all_here = m_storage.set_value(n, p, base, m);
    if (all_here)
      this->schedule_continuation();
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Sets the @c n th result.
  ///
  /// @param n     Result index.
  /// @param value Value to be set.
  ////////////////////////////////////////////////////////////////////
  void set_value(const size_type n, R const& value)
  {
    std::lock_guard<std::mutex> lock{this->m_mtx};
    const bool all_here = m_storage.set_value(n, value);
    if (all_here)
      this->schedule_continuation();
  }

  ////////////////////////////////////////////////////////////////////
  /// @copydoc set_value(const size_type,R const&)
  ////////////////////////////////////////////////////////////////////
  void set_value(const size_type n, R&& value)
  {
    std::lock_guard<std::mutex> lock{this->m_mtx};
    const bool all_here = m_storage.set_value(n, std::move(value));
    if (all_here)
      this->schedule_continuation();
  }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref values_handle for @c void.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<>
class values_handle<void>
: public futures_base<void>
{
public:
  typedef futures_base<void>::size_type size_type;
private:
  typedef async_results<void>           internal_storage_type;

  internal_storage_type m_storage;

public:
  explicit values_handle(const size_type n) noexcept
  : m_storage(n)
  { }

protected:
  bool valid_no_yield(void) const noexcept final
  { return m_storage.valid(); }

public:
  size_type size(void) const noexcept final
  { return m_storage.size(); }

  bool valid(const size_type n) const final
  { return yield_if_not([this, n] { return this->m_storage.valid(n); }); }

  bool valid(void) const final
  { return yield_if_not([this] { return this->m_storage.valid(); }); }

  void wait(const size_type n) const final
  { yield_until([this, n] { return this->m_storage.valid(n); }); }

  void wait(void) const final
  { yield_until([this] { return this->m_storage.valid(); }); }

  void wait(context& ctx) const
  { yield_until(ctx, [this] { return this->m_storage.valid(); }); }

  void get(const size_type n) final
  {
    wait(n);
    std::lock_guard<std::mutex> lock{this->m_mtx};
    m_storage.get(n);
  }

  void get(void) final
  {
    wait();
    std::lock_guard<std::mutex> lock{this->m_mtx};
    m_storage.get();
  }

  void set_value(const size_type n)
  {
    std::lock_guard<std::mutex> lock{this->m_mtx};
    const bool all_here = m_storage.set_value(n);
    if (all_here)
      this->schedule_continuation();
  }
};

} // namespace runtime

} // namespace stapl

#endif
