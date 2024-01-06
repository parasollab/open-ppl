/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_CROSS_TG_NOTIFIER_HPP
#define STAPL_PARAGRAPH_CROSS_TG_NOTIFIER_HPP

#include <stapl/runtime/new.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Function object that delays the execution of a PARAGRAPH,
///   pending notification from predecessor PARAGRAPHs who compute values that
///   the successor receives as view inputs.
/// @ingroup paragraph
///
/// Allows initialization of PARAGRAPH (and perhaps some task initialization to
/// proceed concurrently with predecessor PARAGRAPH execution.
///
/// @sa paragraph::operator()
//////////////////////////////////////////////////////////////////////
struct cross_tg_notifier
{
private:
  /// @brief Number of predecessor PARAGRAPHs this PARAGRAPH is waiting for
  /// notification from before release the associated executor to the parent
  /// executor to make it runnable.
  std::size_t             m_waiting_pred_tg_count;

  /// @brief Pointer to executor of the PARAGRAPH this notifier will notify.
  executor_base*          m_executor_ptr;

  /// @brief Pointer to parent executor of of the PARAGRAPH this notifier
  /// will notify.
  executor_base*          m_parent_executor_ptr;

  cross_tg_notifier(cross_tg_notifier const&);
  cross_tg_notifier& operator=(cross_tg_notifier const&);

public:
  cross_tg_notifier()
    : m_waiting_pred_tg_count(0),
      m_executor_ptr(nullptr),
      m_parent_executor_ptr(nullptr)
  { }

  STAPL_USE_MANAGED_ALLOC(cross_tg_notifier)

  //////////////////////////////////////////////////////////////////////
  /// @brief Receive notification from a predecessor PARAGRAPH.
  ///
  /// Decrement internal notification counter.  If this is the last pending
  /// notification, release the target PARAGRAPH's executor to its parent and
  /// destroy this notifier.
  //////////////////////////////////////////////////////////////////////
  void operator()(void)
  {
    if ((--m_waiting_pred_tg_count == 0) && (m_executor_ptr != nullptr))
    {
      m_parent_executor_ptr->add_executor(m_executor_ptr);

      delete this;
    }
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Initializes executor data members. If all predecessor PARAGRAPH
  /// have already notified, then release the target PARAGRAPH's executor to its
  /// parent and destroy this notifier.
  ///
  /// @param executor_ptr Pointer to executor of the PARAGRAPH this
  ///   notifier will notify.
  /// @param parent_executor_ptr Pointer to parent executor of of the PARAGRAPH
  ///   this notifier will notify.
  ///
  /// PARAGRAPH ensures method is not called until all predecessors have been
  /// accounted for by calls to @p increment_preds.
  //////////////////////////////////////////////////////////////////////
  void set_executor_ptr(executor_base& executor_ptr,
                        executor_base& parent_executor_ptr)
  {
    m_executor_ptr        = &executor_ptr;
    m_parent_executor_ptr = &parent_executor_ptr;

    if (m_waiting_pred_tg_count > 0)
      return;

    // else...
    m_parent_executor_ptr->add_executor(m_executor_ptr);

    delete this;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Increment the predecessor notification count this notifier expects.
  ///
  /// @sa input_edge_available_func
  //////////////////////////////////////////////////////////////////////
  void increment_preds(void)
  {
    ++m_waiting_pred_tg_count;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return true if all notifications from predecessor have been
  /// received and the PARAGRAPH this notifier services can be executed.
  ///
  /// PARAGRAPH ensures method is not called until all predecessors have been
  /// accounted for by calls to @p increment_preds.
  //////////////////////////////////////////////////////////////////////
  bool ready(void) const
  {
    return m_waiting_pred_tg_count == 0;
  }
}; // struct cross_tg_notifier

} // namespace detail

} // namespace stapl

#endif // ifndef STAPL_PARAGRAPH_CROSS_TG_NOTIFIER_HPP

