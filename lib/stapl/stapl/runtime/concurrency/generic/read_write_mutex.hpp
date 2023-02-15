/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONCURRENCY_GENERIC_READ_WRITE_MUTEX_HPP
#define STAPL_RUNTIME_CONCURRENCY_GENERIC_READ_WRITE_MUTEX_HPP

#include "../../exception.hpp"
#include <condition_variable>
#include <mutex>

namespace stapl {

namespace runtime {

namespace generic_impl {

//////////////////////////////////////////////////////////////////////
/// @brief A generic, non-optimized implementation of a read-write mutex.
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
  std::mutex              m_mutex;
  unsigned int            m_readers;
  unsigned int            m_writers;
  bool                    m_writing;
  std::condition_variable m_write_released;
  std::condition_variable m_read_released;

public:
  read_write_mutex(void)
  : m_readers(0),
    m_writers(0),
    m_writing(false)
  { }

  ~read_write_mutex(void)
  {
    STAPL_RUNTIME_CHECK((m_writing==false && m_readers==0 && m_writers==0),
                        "Mutex still in use");
  }

  read_write_mutex(read_write_mutex const&) = delete;
  read_write_mutex& operator=(read_write_mutex const&) = delete;

  void lock(const read_lock_t)
  {
    std::unique_lock<std::mutex> lock(m_mutex);

    // wait for all the write lock requests to finish and the write lock to be
    // released
    while (m_writing || m_writers>0) {
      m_write_released.wait(lock);
    }
    ++m_readers;
  }

  void lock(const write_lock_t = write_lock)
  {
    std::unique_lock<std::mutex> lock(m_mutex);

    ++m_writers;
    while (m_readers>0) { // wait for all read locks to be released
      m_read_released.wait(lock);
    }
    while (m_writing) {   // wait for write lock to be released
      m_write_released.wait(lock);
    }
    --m_writers;
    m_writing = true;
  }

  bool try_lock(const read_lock_t)
  {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_writing || m_writers>0) {
      // write lock or write lock requests waiting - ensures fairness
      return false;
    }
    ++m_readers;
    return true;
  }

  bool try_lock(const write_lock_t = write_lock)
  {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_writing || m_readers>0 || m_writers>0) {
      return false;
    }
    m_writing = true;
    return true;
  }

  void unlock(void)
  {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_readers>0) {
      --m_readers;
      if (m_readers==0) {
        m_read_released.notify_all();
      }
    }
    else if (m_writing) {
      m_writing = false;
      m_write_released.notify_all();
    }
    else {
      STAPL_RUNTIME_ERROR("Not acquired");
    }
  }
};

} // namespace generic_impl


using generic_impl::read_write_mutex;

} // namespace runtime

} // namespace stapl

#endif
