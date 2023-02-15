/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONCURRENCY_INTRUSIVE_MPSC_QUEUE_HPP
#define STAPL_RUNTIME_CONCURRENCY_INTRUSIVE_MPSC_QUEUE_HPP

#include "../config.hpp"
#include "../exception.hpp"
#include <atomic>

namespace stapl {

namespace runtime {

template<typename T>
class intrusive_mpsc_queue;


//////////////////////////////////////////////////////////////////////
/// @brief Hook for @ref intrusive_mpsc_queue.
///
/// @ingroup concurrency
//////////////////////////////////////////////////////////////////////
class intrusive_mpsc_queue_hook
{
private:
  intrusive_mpsc_queue_hook* m_next;

  template<typename T>
  friend class intrusive_mpsc_queue;

public:
  constexpr intrusive_mpsc_queue_hook(void) noexcept
  : m_next(nullptr)
  { }

  intrusive_mpsc_queue_hook(intrusive_mpsc_queue_hook const&) = delete;

  intrusive_mpsc_queue_hook&
  operator=(intrusive_mpsc_queue_hook const&) = delete;
};


//////////////////////////////////////////////////////////////////////
/// @brief An intrusive lock-free MPSC (multiple producer, single consumer)
///        queue.
///
/// @tparam T Object type.
///
/// The objects are kept in an lock-free singly linked list in reversed order.
/// Since this is an intrusive queue, @p T has to inherit from
/// @ref intrusive_mpsc_queue_hook.
///
/// @ingroup concurrency
//////////////////////////////////////////////////////////////////////
template<typename T>
class intrusive_mpsc_queue
{
public:
  typedef T                         value_type;
  typedef value_type const&         const_reference;
  typedef value_type&               reference;
  typedef value_type const*         const_pointer;
  typedef value_type*               pointer;
private:
  typedef intrusive_mpsc_queue_hook node_type;

  /// Singly-linked list.
  STAPL_RUNTIME_CACHELINE_ALIGNED std::atomic<node_type*> m_tail;
  /// Consumer thread head of the list.
  node_type*                                              m_cons_head;

public:
  intrusive_mpsc_queue(void) noexcept
  : m_tail(nullptr),
    m_cons_head(nullptr)
  { }

  intrusive_mpsc_queue(intrusive_mpsc_queue const&) = delete;
  intrusive_mpsc_queue& operator=(intrusive_mpsc_queue const&) = delete;

  bool empty(void) const noexcept
  { return ((m_tail==nullptr) && (m_cons_head==nullptr)); }

  void push(T& t) noexcept
  {
    node_type& n = t;
    STAPL_RUNTIME_ASSERT(!n.m_next);
    node_type* tail = m_tail.load(std::memory_order_relaxed);
    // add new node to the tail of the list; the list is effectively reversed
    do {
      n.m_next = tail;
    }
    while (!m_tail.compare_exchange_weak(tail, &n, std::memory_order_release));
  }

  T* try_pop(void) noexcept
  {
    if (m_cons_head) {
      // the consumer has already elements in its list
      node_type* n = m_cons_head;
      m_cons_head  = m_cons_head->m_next;
      n->m_next    = nullptr;
      return static_cast<T*>(n);
    }

    // get the tail of the list
    node_type* tail = m_tail.exchange(nullptr, std::memory_order_acquire);

    if (!tail)
      return nullptr;

    // reverse the list
    while (tail->m_next) {
      node_type* next = tail->m_next;
      tail->m_next = m_cons_head;
      m_cons_head  = tail;
      tail         = next;
    }

    return static_cast<T*>(tail);
  }
};

} // namespace runtime

} // namespace stapl

#endif
