/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONCURRENCY_TBB_READ_WRITE_MUTEX_HPP
#define STAPL_RUNTIME_CONCURRENCY_TBB_READ_WRITE_MUTEX_HPP

#include <tbb/queuing_rw_mutex.h>

namespace stapl {

namespace runtime {

namespace tbb_impl {

//////////////////////////////////////////////////////////////////////
/// @brief An implementation of a read-write mutex based on
///        @c tbb::queueing_rw_mutex.
///
/// It implements the interface of @c std::mutex, enhanced to support read or
/// write locking.
///
/// The default locking policy is write locking.
///
/// @ingroup concurrency
//////////////////////////////////////////////////////////////////////
class read_write_mutex
{
private:
  tbb::queuing_rw_mutex              m_mutex;
  tbb::queuing_rw_mutex::scoped_lock m_lock;

public:
  read_write_mutex(void) = default;

  read_write_mutex(read_write_mutex const&) = delete;
  read_write_mutex& operator=(read_write_mutex const&) = delete;

  void lock(const read_lock_t)
  { m_lock.acquire(m_mutex, false); }

  void lock(const write_lock_t = write_lock)
  { m_lock.acquire(m_mutex, true); }

  bool try_lock(const read_lock_t)
  { return m_lock.try_acquire(m_mutex, false); }

  bool try_lock(const write_lock_t = write_lock)
  { return m_lock.try_acquire(m_mutex, true); }

  void unlock(void)
  { m_lock.release(); }
};

} // namespace tbb_impl


using tbb_impl::read_write_mutex;

} // namespace runtime

} // namespace stapl

#endif
