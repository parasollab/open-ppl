/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_ASYNC_RESULT_HPP
#define STAPL_RUNTIME_ASYNC_RESULT_HPP

#include "config.hpp"
#include "exception.hpp"
#include "message.hpp"
#include "request/packed_value.hpp"
#include "type_traits/lazy_storage.hpp"
#include "type_traits/is_basic.hpp"
#include "type_traits/is_p_object.hpp"
#include "utility/cache_line_alignment.hpp"
#include <atomic>
#include <type_traits>
#include <utility>
#include <vector>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Storage for a result that can be asynchronously assigned and
///        retrieved.
///
/// @tparam T Result type.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename T, bool IsBasic = is_basic<T>::value>
class async_result
{
private:
  typedef packed_value<T>                    packed_type;
  typedef lazy_storage<T>                    unpacked_type;
public:
  typedef typename packed_type::storage_type storage_type;
private:
  enum status_type
  {
    EMPTY    = 0x0,
    CONSUMED = 0x1,
    PACKED   = 0x2,
    UNPACKED = 0x3
  };

  static_assert(
    (!is_p_object<T>::value                                           &&
    (!std::is_reference<T>::value || is_p_object_reference<T>::value) &&
    (!std::is_pointer<T>::value   || is_p_object_pointer<T>::value)),
    "Returning objects of this type not supported" );

  std::atomic<status_type> m_status;
  union
  {
    packed_type            m_packed;
    unpacked_type          m_unpacked;
  };

public:
  constexpr async_result(void) noexcept
  : m_status(EMPTY),
    m_packed()
  { }

  ~async_result(void)
  {
    const auto status = m_status.load(std::memory_order_acquire);
    switch (status) {
      case EMPTY:
        STAPL_RUNTIME_ERROR("Value not set.");
        break;
      case PACKED:
        m_packed.release();
        break;
      case UNPACKED:
        m_unpacked.destroy();
        break;
      default:
        break;
    }
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns the result.
  ///
  /// Calling @ref valid() const returns @c false after a call to this function.
  ////////////////////////////////////////////////////////////////////
  T get(void)
  {
    const auto status = m_status.exchange(CONSUMED, std::memory_order_acq_rel);
    STAPL_RUNTIME_ASSERT(status==PACKED || status==UNPACKED);
    return (status==PACKED ? m_packed.get() : m_unpacked.moveout());
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns if a result is stored.
  ////////////////////////////////////////////////////////////////////
  bool valid(void) const noexcept
  {
    const auto status = m_status.load(std::memory_order_relaxed);
    return (status==PACKED || status==UNPACKED);
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Sets the result.
  ///
  /// This function keeps a pointer to the buffer where the result remains
  /// packed until @ref get() is called.
  ///
  /// @param p    Pointer to the @ref arg_storage in the buffer.
  /// @param base Address in the buffer the @ref arg_storage was created in.
  /// @param m    Buffer that contains the @ref arg_storage.
  ///
  /// @see arg_storage, message
  ////////////////////////////////////////////////////////////////////
  void set_value(storage_type* const p,
                 void* const base,
                 message_shared_ptr& m) noexcept
  {
    STAPL_RUNTIME_ASSERT(!valid());
    m_packed.set(p, base, m);
    m_status.store(PACKED, std::memory_order_release);
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Sets the result.
  ////////////////////////////////////////////////////////////////////
  void set_value(T const& value)
  {
    STAPL_RUNTIME_ASSERT(!valid());
    const auto r = typer_traits<T>::meets_requirements(typer::COPY, value);
    if (r.first) {
      // value can be copied in shared memory
      m_unpacked.construct(value);
      m_status.store(UNPACKED, std::memory_order_release);
    }
    else {
      // value cannot be copied in shared memory
      m_packed.set(value);
      m_status.store(PACKED, std::memory_order_release);
    }
  }

  ////////////////////////////////////////////////////////////////////
  /// @copydoc set_value(T const&)
  ////////////////////////////////////////////////////////////////////
  void set_value(T&& value)
  {
    STAPL_RUNTIME_ASSERT(!valid());
    const auto r = typer_traits<T>::meets_requirements(typer::MOVE, value);
    if (r.first) {
      // value can be moved in shared memory
      m_unpacked.construct(std::move(value));
      m_status.store(UNPACKED, std::memory_order_release);
    }
    else {
      // value cannot be moved in shared memory but move it into packed_value
      m_packed.set(std::move(value));
      m_status.store(PACKED, std::memory_order_release);
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref async_result for basic types.
///
/// @see is_basic
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename T>
class async_result<T, true>
{
private:
  typedef packed_value<T>                    packed_type;
  typedef lazy_storage<T>                    internal_storage_type;
public:
  typedef typename packed_type::storage_type storage_type;
private:
  enum status_type
  {
    EMPTY    = 0x0,
    CONSUMED = 0x1,
    SET      = 0x2
  };

  std::atomic<status_type> m_status;
  internal_storage_type    m_storage;

public:
  constexpr async_result(void) noexcept
  : m_status(EMPTY)
  { }

  ~async_result(void)
  {
    const status_type status = m_status.load(std::memory_order_acquire);
    switch (status) {
      case EMPTY:
        STAPL_RUNTIME_ERROR("Value not set.");
        break;
      case SET:
        m_storage.destroy();
        break;
      default:
        break;
    }
  }

  T get(void)
  {
    STAPL_RUNTIME_ASSERT(m_status==SET);
    m_status.store(CONSUMED, std::memory_order_release);
    return m_storage.moveout();
  }

  bool valid(void) const noexcept
  { return (m_status.load(std::memory_order_relaxed)==SET); }

  void set_value(storage_type* const p,
                 void* const base,
                 message_shared_ptr& m) noexcept
  {
    STAPL_RUNTIME_ASSERT(!valid());
    m_storage.construct(packed_type{p, base, m}.get());
    m_status.store(SET, std::memory_order_release);
  }

  void set_value(T const& value)
  {
    STAPL_RUNTIME_ASSERT(!valid());
    m_storage.construct(value);
    m_status.store(SET, std::memory_order_release);
  }

  void set_value(T&& value)
  {
    STAPL_RUNTIME_ASSERT(!valid());
    m_storage.construct(std::move(value));
    m_status.store(SET, std::memory_order_release);
  }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref async_result for @c void.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<>
class async_result<void>
{
private:
  enum status_type
  {
    EMPTY    = 0x0,
    SET      = 0x1,
    CONSUMED = 0x2
  };

  std::atomic<status_type> m_status;

public:
  constexpr async_result(void) noexcept
  : m_status(EMPTY)
  { }

  ~async_result(void)
  { STAPL_RUNTIME_ASSERT(m_status.load(std::memory_order_relaxed)!=EMPTY); }

  void get(void) noexcept
  {
    STAPL_RUNTIME_ASSERT(valid());
    m_status.store(CONSUMED, std::memory_order_relaxed);
  }

  bool valid(void) const noexcept
  { return (m_status.load(std::memory_order_relaxed)==SET); }

  void set_value(void) noexcept
  {
    STAPL_RUNTIME_ASSERT(!valid());
    m_status.store(SET, std::memory_order_relaxed);
  }
};



//////////////////////////////////////////////////////////////////////
/// @brief Storage for an array of results that can be asynchronously assigned
///        and retrieved.
///
/// @tparam T Result type.
///
/// @ingroup requestBuildingBlock
///
/// @todo Size of storage is not dynamic, use array / unique_ptr vs vector.
//////////////////////////////////////////////////////////////////////
template<typename T>
class async_results
{
private:
  typedef cache_line_aligned_vector<async_result<T>> container_type;

public:
  typedef typename container_type::size_type         size_type;
  typedef typename async_result<T>::storage_type     storage_type;

private:
  container_type         m_storage;
  std::atomic<size_type> m_cnt;

public:
  explicit async_results(const size_type n)
  : m_storage(n),
    m_cnt(n)
  { }

  size_type size(void) const noexcept
  { return m_storage.size(); }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns the n-th result.
  ///
  /// Calling @ref valid(size_type) const or @ref valid() const returns @c false
  /// after a call to this function.
  ////////////////////////////////////////////////////////////////////
  T get(const size_type n)
  {
    m_cnt.fetch_add(1, std::memory_order_relaxed);
    return m_storage[n].get();
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns all the results.
  ///
  /// Calling @ref valid(size_type) const or @ref valid() const returns @c false
  /// after a call to this function.
  ////////////////////////////////////////////////////////////////////
  std::vector<T> get(void)
  {
    std::vector<T> v;
    STAPL_RUNTIME_ASSERT(valid());
    m_cnt.fetch_add(m_storage.size(), std::memory_order_relaxed);
    v.reserve(size());
    for (size_type i=0; i<size(); ++i)
      v.push_back(m_storage[i].get());
    return v;
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns if the n-th result is stored.
  ////////////////////////////////////////////////////////////////////
  bool valid(const size_type n) const noexcept
  { return m_storage[n].valid(); }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns if all the results are stored.
  ////////////////////////////////////////////////////////////////////
  bool valid(void) const noexcept
  { return (m_cnt.load(std::memory_order_relaxed)==0); }

  ////////////////////////////////////////////////////////////////////
  /// @brief Sets the n-th result and returns @c true if it was the last result
  ///        expected.
  ///
  /// This function keeps a pointer to the buffer where the result remains
  /// packed until @ref get() or @ref get(const size_type) is called.
  ///
  /// @param n    Result index.
  /// @param p    Pointer to the @ref arg_storage in the buffer.
  /// @param base Address in the buffer the @ref arg_storage was created in.
  /// @param m    Buffer that contains the @ref arg_storage.
  ///
  /// @see arg_storage, message
  ////////////////////////////////////////////////////////////////////
  bool set_value(const size_type n,
                 storage_type* const p,
                 void* const base,
                 message_shared_ptr& m) noexcept
  {
    STAPL_RUNTIME_ASSERT(!valid());
    m_storage[n].set_value(p, base, m);
    return (m_cnt.fetch_sub(1, std::memory_order_relaxed)==1);
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Sets the n-th result and returns @c true if it was the last result
  ///        expected.
  ////////////////////////////////////////////////////////////////////
  bool set_value(const size_type n, T const& value)
  {
    STAPL_RUNTIME_ASSERT(!valid());
    m_storage[n].set_value(value);
    return (m_cnt.fetch_sub(1, std::memory_order_relaxed)==1);
  }

  ////////////////////////////////////////////////////////////////////
  /// @copydoc set_value(const size_type,T const&)
  ////////////////////////////////////////////////////////////////////
  bool set_value(const size_type n, T&& value)
  {
    STAPL_RUNTIME_ASSERT(!valid());
    m_storage[n].set_value(std::move(value));
    return (m_cnt.fetch_sub(1, std::memory_order_relaxed)==1);
  }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref async_result for @c void.
///
/// @ingroup requestBuildingBlock
///
/// @todo Size of storage is not dynamic, use array / unique_ptr vs vector.
//////////////////////////////////////////////////////////////////////
template<>
class async_results<void>
{
private:
  typedef cache_line_aligned_vector<async_result<void>> container_type;

public:
  typedef typename container_type::size_type            size_type;

private:
  container_type         m_storage;
  std::atomic<size_type> m_cnt;

public:
  explicit async_results(const size_type n)
  : m_storage(n),
    m_cnt(n)
  { }

  size_type size(void) const noexcept
  { return m_storage.size(); }

  void get(const size_type n) noexcept
  {
    m_cnt.fetch_add(1, std::memory_order_relaxed);
    m_storage[n].get();
  }

  void get(void) noexcept
  {
    STAPL_RUNTIME_ASSERT(valid());
    m_cnt.fetch_add(m_storage.size(), std::memory_order_relaxed);
    for (size_type i=0; i<size(); ++i)
      m_storage[i].get();
  }

  bool valid(const size_type n) const noexcept
  { return m_storage[n].valid(); }

  bool valid(void) const noexcept
  { return (m_cnt.load(std::memory_order_relaxed)==0); }

  bool set_value(const size_type n) noexcept
  {
    STAPL_RUNTIME_ASSERT(!valid());
    m_storage[n].set_value();
    return (m_cnt.fetch_sub(1, std::memory_order_relaxed)==1);
  }
};

} // namespace runtime

} // namespace stapl

#endif
