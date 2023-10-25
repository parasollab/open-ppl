/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONCURRENCY_TASK_QUEUE_HPP
#define STAPL_RUNTIME_CONCURRENCY_TASK_QUEUE_HPP

#include <atomic>
#include <utility>

namespace stapl {

namespace runtime {

template<typename>
class task_queue;

//////////////////////////////////////////////////////////////////////
/// @brief A lock-free MPSC (multiple producer, single consumer) queue of tasks
///        to be executed.
///
/// The tasks are kept in an lock-free singly linked list in reversed order.
/// Each node of the list is heap-allocated, since the type of the task is
/// type-erased.
///
/// When @ref drain(args...) is called, the list is reversed and the tasks are
/// called with @p args... in the same order they were added to the queue.
///
/// @ingroup concurrency
//////////////////////////////////////////////////////////////////////
template<typename R, typename... Args>
class task_queue<R(Args...)>
{
private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Node type base class.
  //////////////////////////////////////////////////////////////////////
  struct node_base
  {
    node_base* m_next;

    constexpr node_base(void) noexcept
    : m_next(nullptr)
    { }

    virtual ~node_base(void) = default;

    virtual void operator()(Args...) = 0;
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Node type for the linked list.
  ///
  /// @tparam Function Function type to be called.
  //////////////////////////////////////////////////////////////////////
  template<typename Function>
  struct node final
  : public node_base,
    private Function
  {
    template<typename F>
    explicit node(F&& f)
    : Function(std::forward<F>(f))
    { }

    void operator()(Args... args)
    { static_cast<Function&>(*this)(args...); }
  };

  std::atomic<node_base*> m_tail;

public:
  task_queue(void) noexcept
  : m_tail(nullptr)
  { }

  bool empty(void) const noexcept
  { return (m_tail==nullptr); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds a new task to the queue.
  //////////////////////////////////////////////////////////////////////
  template<typename Function>
  void add(Function&& f)
  {
    node<Function>* n = new node<Function>(std::forward<Function>(f));
    node_base* tail = m_tail.load(std::memory_order_relaxed);
    // add it to the head of the list
    do {
      n->m_next = tail;
    }
    while (!m_tail.compare_exchange_weak(tail, n, std::memory_order_release));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Executes all pending tasks.
  ///
  /// This function will only execute the pending tasks when the function is
  /// called. If any tasks are added while @ref drain() is executing, then they
  /// will not be executed.
  ///
  /// @param t Arguments to pass to the pending tasks.
  ///
  /// @return @c true if there were pending tasks, otherwise @c false.
  //////////////////////////////////////////////////////////////////////
  template<typename... T>
  bool drain(T&&... t)
  {
    // get the tail of the list
    node_base* tail = m_tail.exchange(nullptr, std::memory_order_acquire);

    if (!tail)
      return false;

    // reverse the list
    node_base* head = nullptr;
    do {
      node_base* next = tail->m_next;
      tail->m_next = head;
      head = tail;
      tail = next;
    } while (tail);

    // process it in order
    do {
      (*head)(t...);
      node_base* next = head->m_next;
      delete head;
      head = next;
    } while (head);

    return true;
  }
};

} // namespace runtime

} // namespace stapl

#endif
