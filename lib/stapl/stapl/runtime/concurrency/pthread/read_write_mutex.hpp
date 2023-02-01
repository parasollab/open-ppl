/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONCURRENCY_PTHREAD_READ_WRITE_MUTEX_HPP
#define STAPL_RUNTIME_CONCURRENCY_PTHREAD_READ_WRITE_MUTEX_HPP

#include "../../exception.hpp"
#include <pthread.h>

namespace stapl {

namespace runtime {

namespace phtread_impl {

//////////////////////////////////////////////////////////////////////
/// @brief A read-write mutex based on @c pthread_rwlock_t.
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
  pthread_rwlock_t m_mutex;

public:
  read_write_mutex(void) noexcept
  {
    STAPL_RUNTIME_CHECK((pthread_rwlock_init(&m_mutex,NULL)==0),
                        "Initialization failed");
  }

  ~read_write_mutex(void) noexcept
  {
    STAPL_RUNTIME_CHECK((pthread_rwlock_destroy(&m_mutex)==0),
                        "Initialization failed");
  }

  read_write_mutex(read_write_mutex const&) = delete;
  read_write_mutex& operator=(read_write_mutex const&) = delete;

  void lock(const read_lock_t) noexcept
  { STAPL_RUNTIME_CHECK((pthread_rwlock_rdlock(&m_mutex)==0), "Failed"); }

  void lock(const write_lock_t = write_lock) noexcept
  { STAPL_RUNTIME_CHECK((pthread_rwlock_wrlock(&m_mutex)==0), "Failed"); }

  bool try_lock(const read_lock_t) noexcept
  { return (pthread_rwlock_tryrdlock(&m_mutex)==0); }

  bool try_lock(const write_lock_t = write_lock) noexcept
  { return (pthread_rwlock_trywrlock(&m_mutex)==0); }

  void unlock(void) noexcept
  { STAPL_RUNTIME_CHECK((pthread_rwlock_unlock(&m_mutex)==0), "Failed"); }
};

} // namespace phtread_impl


using phtread_impl::read_write_mutex;

} // namespace runtime

} // namespace stapl

#endif
