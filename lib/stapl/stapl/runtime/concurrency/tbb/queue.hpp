/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONCURRENCY_TBB_QUEUE_HPP
#define STAPL_RUNTIME_CONCURRENCY_TBB_QUEUE_HPP

#include <tbb/concurrent_queue.h>

namespace stapl {

namespace runtime {

namespace tbb_impl {

//////////////////////////////////////////////////////////////////////
/// @brief A thread-safe queue using @c tbb::concurrent_queue.
///
/// @ingroup concurrency
//////////////////////////////////////////////////////////////////////
template<typename T>
class queue
{
private:
  typedef tbb::concurrent_queue<T>             queue_type;
public:
  typedef typename queue_type::value_type      value_type;
  typedef typename queue_type::reference       reference;
  typedef typename queue_type::const_reference const_reference;
  typedef typename queue_type::size_type       size_type;

private:
  queue_type m_queue;

public:
  queue& operator=(queue&& q)
  {
    queue_type t(std::move(q.m_queue));
    m_queue.clear();
    for (auto it = m_queue.unsafe_begin(); it != m_queue.unsafe_end(); ++it) {
      m_queue.push(*it);
    }
    return *this;
  }

  bool empty(void) const noexcept
  { return m_queue.empty(); }

  size_type size(void) const noexcept
  { return m_queue.unsafe_size(); }

  void push(value_type const& x)
  { m_queue.push(x); }

  void push(value_type&& x)
  { m_queue.push(std::move(x)); }

  bool try_pop(value_type& x)
  { return m_queue.try_pop(x); }

  void clear(void)
  { m_queue.clear(); }
};

} // namespace tbb_impl


using tbb_impl::queue;

} // namespace runtime

} // namespace stapl

#endif
