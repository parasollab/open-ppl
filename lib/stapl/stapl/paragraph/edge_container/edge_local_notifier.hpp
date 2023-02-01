/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_EDGE_LOCAL_NOTIFIER_HPP
#define STAPL_PARAGRAPH_EDGE_LOCAL_NOTIFIER_HPP

#include <boost/intrusive_ptr.hpp>

namespace stapl {

namespace detail {

using paragraph_impl::task_base;

//////////////////////////////////////////////////////////////////////
/// @brief Represents the primary interface through which users of
///   the @ref edge_container or @ref edge_view (i.e., @p task_graph) are
///   informed of data flow events triggered by the completion of predecessor
///    tasks.
/// @ingroup pgEdgeNotifiers
///
/// Instances of this class are heap allocated and passed to the
/// @ref edge_container when successor tasks create an edge via
/// @ref edge_view::setup_flow.  When data flow is available for a successor,
/// the @p edge_container will invoke this object's function operator, once
/// per specified predecessor.  When all notifications have been received,
/// this object passes the successor task to the PARAGRAPH's executor, making it
/// eligible for execution.
///
/// Note that the interface to access the value facilitated through a
/// different interface, namely @ref edge_view::operator[].
///
/// @sa edge_view::setup_flow
/// @sa edge_view::operator[]
/// @sa aggregated_edge_view::setup_flow
/// @sa aggregated_edge_view::operator[]
/// @sa executor_base
//////////////////////////////////////////////////////////////////////
struct edge_local_notifier_base
{
protected:
  /// @brief Denotes whether PARAGRAPH this edge notifier is a part of is
  /// persistent.
  const bool m_b_persistent;

public:
  typedef void result_type;

  edge_local_notifier_base(bool persistent)
    : m_b_persistent(persistent)
  { }

  virtual ~edge_local_notifier_base(void) = default;


  //////////////////////////////////////////////////////////////////////
  /// @brief Invoked by each predecessor edge which this notifier was registered
  /// with, signifying that the value is now available for consumption.  When
  /// the @p operator() has been called by all predecessors, the task associated
  /// with the notifier (@p m_task_ptr) is released to the @p executor as
  /// runnable.
  //////////////////////////////////////////////////////////////////////
  virtual void operator()(executor_base&) = 0;


  //////////////////////////////////////////////////////////////////////
  /// @brief Set the internal pointer to the task associated with this notifier
  /// so that it can be passed to the executor when all edge trigger events
  /// (i.e., operator() calls) have occurred.
  ///
  /// @param t        Pointer to task associated with this predecessor notifier.
  /// @param executor The executor associated with the PARAGRAPH for this
  ///                 notifier.
  ///
  /// Initialization of the task happens concurrently with the construction of
  /// this object and the edge creations it participates in.  When
  /// @p task_graph_impl::add_task finishes creating the task, it calls
  /// this method.
  ///
  /// @sa task_graph_impl::add_task
  //////////////////////////////////////////////////////////////////////
  virtual void set_task_ptr(task_base* t, executor_base& executor) = 0;


  //////////////////////////////////////////////////////////////////////
  /// @brief Freestanding function required for boost::intrusive_ptr, which is
  /// used to implement reference counting on this object, as needed for
  /// persistent PARAGRAPH usage.
  ///
  /// @param ptr An instance of @p edge_local_notifier_base who reference count
  /// is to be incremented.
  //////////////////////////////////////////////////////////////////////
  friend
  void intrusive_ptr_add_ref(edge_local_notifier_base const* ptr)
  {
    ptr->intrusive_ptr_add_ref_impl();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Freestanding function required for boost::intrusive_ptr, which is
  /// used to implement reference counting on this object, as needed for
  /// persistent PARAGRAPH usage.
  ///
  /// @param ptr An instance of @p edge_local_notifier_base who reference count
  ///   is to be decremented.
  //////////////////////////////////////////////////////////////////////
  friend
  void intrusive_ptr_release(edge_local_notifier_base const* ptr)
  {
    ptr->intrusive_ptr_release_impl();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Pure virtual method that defines interface for implementation of
  /// counter increment called by @p freestanding @ref intrusive_ptr_add_ref.
  /// Implemented in @ref edge_local_notifier.
  //////////////////////////////////////////////////////////////////////
  virtual void intrusive_ptr_add_ref_impl(void) const = 0;


  //////////////////////////////////////////////////////////////////////
  /// @brief Pure virtual method that defines interface for implementation of
  /// counter decrement called by freestanding @ref intrusive_ptr_release.
  /// Implemented in @ref edge_local_notifier.
  //////////////////////////////////////////////////////////////////////
  virtual void intrusive_ptr_release_impl(void) const = 0;
};


//////////////////////////////////////////////////////////////////////
/// @brief Derived class of @p edge_local_notifier_base that holds notifier
///   counters, associated task and metadata for scheduling.  Type is erased
///   immediately after construction in @ref task_graph_impl::add_task.
/// @ingroup pgEdgeNotifiers
///
/// @todo Track down why @p m_ref_count is mutable.  Probably tied to
/// overly conservative const qualification of notifiers in the
/// @p edge_container.
//////////////////////////////////////////////////////////////////////
struct edge_local_notifier
  : public edge_local_notifier_base
{
private:
  /// @brief The count of outstanding predecessor notifications for this
  /// execution of the PARAGRAPH.
  std::size_t          m_pred_count;

  /// @brief Pointer to the task this notifier should make runnable when all
  /// predecessors have triggered data flow.
  task_base*           m_task_ptr;

  /// @brief The in-degree of the associated task in the PARAGRAPH. Used to
  /// reinitialize m_pred_count when persistent reexecution enabled.
  const std::size_t    m_init_count;

  /// @brief Used to track the number of predecessor @p edge_entry notifier list
  /// that have a reference to this object.  Used for lifetime management when
  /// used in a persistent PARAGRAPH (Otherwise, deletion in operator()).
  mutable std::size_t  m_ref_count;

  edge_local_notifier(edge_local_notifier const&) = delete;
  edge_local_notifier& operator=(edge_local_notifier const&) = delete;

public:
  explicit
  edge_local_notifier(std::size_t pred_count, bool persistent)
    : edge_local_notifier_base(persistent),
      m_pred_count(pred_count),
      m_task_ptr(nullptr),
      m_init_count(pred_count),
      m_ref_count(0)
  { }


  ~edge_local_notifier() final
  {
    if (m_b_persistent)
    {
      delete m_task_ptr;
    }
  }


  STAPL_USE_MANAGED_ALLOC(edge_local_notifier)


  //////////////////////////////////////////////////////////////////////
  /// @brief Implementation of pure virtual method defined in abstract base
  /// class @p edge_local_notifier.
  ///
  /// Increment reference count and check
  /// assertions context on which method was called.
  ///
  /// @sa edge_local_notifier::intrusive_ptr_add_ref_impl
  //////////////////////////////////////////////////////////////////////
  void intrusive_ptr_add_ref_impl(void) const final
  {
    stapl_assert(m_b_persistent,
      "Intrusive add_ref called when not persistent");

    ++m_ref_count;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Implementation of pure virtual method defined in abstract base
  /// class @p edge_local_notifier.
  ///
  /// Decrement reference count and check
  /// assertions context on which method was called.
  ///
  /// @sa edge_local_notifier::intrusive_ptr_release_impl
  //////////////////////////////////////////////////////////////////////
  void intrusive_ptr_release_impl(void) const final
  {
    stapl_assert(m_b_persistent,
      "Intrusive release called when not persistent");

    if (--m_ref_count == 0)
    {
      stapl_assert(m_pred_count == m_init_count,
        "Unexpected m_pred_count in ptr_release");

      delete this;
    }
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Implementation of pure virtual method defined in abstract base
  /// class @p edge_local_notifier.
  ///
  /// @sa edge_local_notifier::operator()
  //////////////////////////////////////////////////////////////////////
  void operator()(executor_base& executor) final
  {
    stapl_assert(m_pred_count,
      "edge_local_notifier::operator(): invoked with m_pred_count == 0");

    // m_task_ptr isn't set until the the task is fully constructed and
    // all predecessor info is initialized.  If pred_count is reduced to
    // 0 before m_task_ptr is set, it will be added to the executor in
    // set_task_ptr (below) when invoked at the end tg:add_task()
    if ((--m_pred_count == 0) && (m_task_ptr != nullptr))
    {
      executor.add_task(m_task_ptr);

      /// reset if persistent, otherwise self destruct.
      if (this->m_b_persistent)
        m_pred_count = m_init_count;
      else
      {
        delete this;
      }
    }
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Implementation of pure virtual method defined in abstract base
  /// class @p edge_local_notifier.
  ///
  /// @sa edge_local_notifier::set_task_ptr
  //////////////////////////////////////////////////////////////////////
  void set_task_ptr(task_base* t, executor_base& executor) final
  {
    m_task_ptr = t;

    // I'm trying to catch underflow and wrap around to large size_t values
    // This assume< 5000 view parameters to a workfunction :)
    //
    stapl_assert(m_pred_count <= 5000,
      "edge_local_notifier::set_task_ptr: found m_pred_count underflow");

    // I still have outstanding preds.  Subsequent calls to operator()
    // will decrement count and eventually make me runnable.
    //
    if (m_pred_count > 0)
      return;

    // else...
    //
    // I had preds, otherwise this edge_local_notifier object wouldn't have
    // been created. However, they have finished and notified me during task
    // initialization (i.e., in tg::add_task() between
    // edge_local_notifier::edge_local_notifier(...) and this call). notify
    // executor task is runnable and kill myself.
    //
    executor.add_task(m_task_ptr);

    if (this->m_b_persistent)
      m_pred_count = m_init_count;
    else
      delete this;

  }
}; // struct edge_local_notifier

} // namespace detail

} // namespace stapl

#endif // ifndef STAPL_PARAGRAPH_EDGE_LOCAL_NOTIFIER_HPP

