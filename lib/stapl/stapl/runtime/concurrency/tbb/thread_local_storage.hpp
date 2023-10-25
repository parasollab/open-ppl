/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONCURRENCY_TBB_THREAD_LOCAL_STORAGE_HPP
#define STAPL_RUNTIME_CONCURRENCY_TBB_THREAD_LOCAL_STORAGE_HPP

#include <tbb/enumerable_thread_specific.h>

namespace stapl {

namespace runtime {

namespace tbb_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Implements thread-local storage using
///        @c tbb::enumerable_thread_specific.
///
/// @ingroup concurrency
//////////////////////////////////////////////////////////////////////
template<typename T>
class thread_local_storage
{
private:
  tbb::enumerable_thread_specific<T> m_t;

public:
  thread_local_storage(void) = default;

  ~thread_local_storage(void)
  { m_t.clear(); }

  thread_local_storage(thread_local_storage const&) = delete;
  thread_local_storage& operator=(thread_local_storage const&) = delete;

  T& get(void)
  { return m_t.local(); }
};

} // namespace tbb_impl

} // namespace runtime

} // namespace stapl


//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
/// @brief Thread-local storage variable definition helper macro for
///        @ref stapl::runtime::tbb_impl::thread_local_storage.
///
/// @ingroup concurrency
//////////////////////////////////////////////////////////////////////
#define STAPL_RUNTIME_THREAD_LOCAL(T, n) \
 stapl::runtime::tbb_impl::thread_local_storage<T> n;

#endif
