/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONCURRENCY_MUTEX_HPP
#define STAPL_RUNTIME_CONCURRENCY_MUTEX_HPP

#include "config.hpp"
#include "../config/platform.hpp"
#include <atomic>
#include <mutex>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Spin-based synchronization primitive that does not emit memory
///        fences.
///
/// @ingroup concurrency
//////////////////////////////////////////////////////////////////////
class thin_spin_mutex
{
public:
  typedef std::atomic_flag& native_handle_type;

private:
  STAPL_RUNTIME_CACHELINE_ALIGNED std::atomic_flag m_lock;

public:
  thin_spin_mutex(void) noexcept
  { m_lock.clear(std::memory_order_relaxed); }

  void lock(void)
  {
    while (!try_lock())
      ; // spin until lock acquired
  }

  bool try_lock(void)
  { return !m_lock.test_and_set(std::memory_order_relaxed); }

  void unlock(void)
  { m_lock.clear(std::memory_order_relaxed); }

  native_handle_type native_handle(void) noexcept
  { return m_lock; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Hierarchical synchronization primitive that does not emit memory
///        fences.
///
/// @ingroup concurrency
//////////////////////////////////////////////////////////////////////
class hierarchical_mutex
{
private:
  thin_spin_mutex           m_lock;
  hierarchical_mutex* const m_parent;

public:
  explicit hierarchical_mutex(hierarchical_mutex* const p = nullptr) noexcept
  : m_parent(p)
  { }

  void lock(void)
  {
    while (!m_lock.try_lock())
      ; // spin until lock acquired

    if (m_parent)
      m_parent->lock();
  }

  bool try_lock(void)
  {
    if (!m_lock.try_lock())
      return false;

    if (m_parent) {
      if (!m_parent->try_lock()) {
        m_lock.unlock();
        return false;
      }
    }
    return true;
  }

  void unlock(void)
  {
    m_lock.unlock();
    if (m_parent)
      m_parent->unlock();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Tag type for read lock ownership.
///
/// @ingroup concurrency
//////////////////////////////////////////////////////////////////////
struct read_lock_t
{ };

//////////////////////////////////////////////////////////////////////
/// @brief Tag for read lock ownership.
///
/// @ingroup concurrency
//////////////////////////////////////////////////////////////////////
constexpr read_lock_t read_lock = { };


//////////////////////////////////////////////////////////////////////
/// @brief Tag type for write lock ownership.
///
/// @ingroup concurrency
//////////////////////////////////////////////////////////////////////
struct write_lock_t
{ };

//////////////////////////////////////////////////////////////////////
/// @brief Tag for write lock ownership.
///
/// @ingroup concurrency
//////////////////////////////////////////////////////////////////////
constexpr write_lock_t write_lock = { };


//////////////////////////////////////////////////////////////////////
/// @brief Lock guard for read lock ownership.
///
/// If write lock is required, then @c lock_guard should be used.
///
/// @ingroup concurrency
//////////////////////////////////////////////////////////////////////
template<typename Mutex>
struct read_lock_guard
{
  Mutex& m_mtx;

  read_lock_guard(Mutex& mtx)
  : m_mtx(mtx)
  { m_mtx.lock(read_lock); }

  ~read_lock_guard(void)
  { m_mtx.unlock(); }
};

} // namespace runtime

} // namespace stapl


#if defined(STAPL_RUNTIME_TBB_AVAILABLE)
// use TBB optimized read-write mutex
# include "tbb/read_write_mutex.hpp"
#elif defined(STAPL_RUNTIME_WINDOWS_TARGET)
// use generic implementation
# include "generic/read_write_mutex.hpp"
#else
// pthreads should be available, use pthreads-based read-write lock
# include "pthread/read_write_mutex.hpp"
#endif

#endif
