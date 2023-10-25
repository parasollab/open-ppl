/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONCURRENCY_BOOST_THREAD_LOCAL_STORAGE_HPP
#define STAPL_RUNTIME_CONCURRENCY_BOOST_THREAD_LOCAL_STORAGE_HPP

#include <boost/thread/tss.hpp>

namespace stapl {

namespace runtime {

namespace boost_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Implements thread-local storage using @c boost::thread_specific_ptr.
///
/// @warning This is often a slower performing thread-local storage than
///          @c thread_local, but is required for compilers and platforms that
///          do not support the latter.
///
/// @ingroup concurrency
//////////////////////////////////////////////////////////////////////
template<typename T>
class thread_local_storage
{
private:
  boost::thread_specific_ptr<T> m_t;

public:
  thread_local_storage(void) = default;

  thread_local_storage(thread_local_storage const&) = delete;
  thread_local_storage& operator=(thread_local_storage const&) = delete;

  ~thread_local_storage(void)
  { m_t.reset(); }

  T& get(void)
  {
    T* p = m_t.get();
    if (!p) {
      p = new T();
      m_t.reset(p);
    }
    return *p;
  }
};

} // namespace boost_impl

} // namespace runtime

} // namespace stapl


//////////////////////////////////////////////////////////////////////
/// @brief Thread-local storage variable definition helper macro for
///        @ref stapl::runtime::boost_impl::thread_local_storage.
///
/// @ingroup concurrency
//////////////////////////////////////////////////////////////////////
#define STAPL_RUNTIME_THREAD_LOCAL(t, x) \
 stapl::runtime::boost_impl::thread_local_storage<t> x;

#endif
