/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONCURRENCY_GENERIC_THREAD_LOCAL_STORAGE_HPP
#define STAPL_RUNTIME_CONCURRENCY_GENERIC_THREAD_LOCAL_STORAGE_HPP

namespace stapl {

namespace runtime {

namespace generic_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Helper class for thread-local storage through C++11 @c thread_local.
///
/// @ingroup concurrency
//////////////////////////////////////////////////////////////////////
template<typename T>
class thread_local_storage
{
private:
  T m_t;

public:
  thread_local_storage(void)
  : m_t()
  { }

  T& get(void) noexcept
  { return m_t; }
};

} // namespace generic_impl

} // namespace runtime

} // namespace stapl


//////////////////////////////////////////////////////////////////////
/// @brief Thread-local storage variable definition helper macro for
///        @ref stapl::runtime::generic_impl::thread_local_storage.
///
/// @ingroup concurrency
//////////////////////////////////////////////////////////////////////
#define STAPL_RUNTIME_THREAD_LOCAL(T, n) \
  thread_local stapl::runtime::generic_impl::thread_local_storage<T> n;

#endif
