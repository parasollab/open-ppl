/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONCURRENCY_GENERIC_QUEUE_HPP
#define STAPL_RUNTIME_CONCURRENCY_GENERIC_QUEUE_HPP

#include <deque>
#include <mutex>
#include <utility>

namespace stapl {

namespace runtime {

namespace generic_impl {

//////////////////////////////////////////////////////////////////////
/// @brief A thread-safe queue.
///
/// @warning Thread-safety is enforced through locks, therefore this queue is
///          not the best performing.
///
/// @ingroup concurrency
//////////////////////////////////////////////////////////////////////
template<typename T>
class queue
{
private:
  typedef std::deque<T>                   queue_type;
public:
  typedef typename queue_type::value_type value_type;
  typedef value_type&                     reference;
  typedef value_type const&               const_reference;
  typedef typename queue_type::size_type  size_type;

private:
  queue_type         m_queue;
  mutable std::mutex m_mtx;

public:
  queue(void) = default;

  queue(queue const& other)
  {
    std::lock_guard<std::mutex> lock{other.m_mtx};
    m_queue = other.m_queue;
  }

  queue(queue&& other)
  : m_queue(std::move(other.m_queue))
  { }

  queue& operator=(queue const& other)
  {
    if (this!=&other) {
      queue tmp{other}; // copy-and-swap idiom to avoid deadlock
      std::lock_guard<std::mutex> lock{m_mtx};
      m_queue.swap(tmp.m_queue);
    }
    return *this;
  }

  queue& operator=(queue&& other)
  {
    std::lock_guard<std::mutex> lock{m_mtx};
    m_queue = std::move(other.m_queue);
    return *this;
  }

  bool empty(void) const
  {
    std::lock_guard<std::mutex> lock{m_mtx};
    return m_queue.empty();
  }

  size_type size(void) const
  {
    std::lock_guard<std::mutex> lock{m_mtx};
    return m_queue.size();
  }

  void push(value_type const& x)
  {
    std::lock_guard<std::mutex> lock{m_mtx};
    m_queue.push_back(x);
  }

  void push(value_type&& x)
  {
    std::lock_guard<std::mutex> lock{m_mtx};
    m_queue.push_back(std::move(x));
  }

  bool try_pop(value_type& x)
  {
    std::lock_guard<std::mutex> lock{m_mtx};
    if (m_queue.empty())
      return false;
    x = m_queue.front();
    m_queue.pop_front();
    return true;
  }

  void clear(void)
  {
    std::lock_guard<std::mutex> lock{m_mtx};
    m_queue.clear();
  }
};

} // namespace generic_impl


using generic_impl::queue;

} // namespace runtime

} // namespace stapl

#endif
