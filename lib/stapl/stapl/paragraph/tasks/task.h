/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_TASK_H
#define STAPL_PARAGRAPH_TASK_H

#include <stapl/runtime/runtime_fwd.hpp>
#include <stapl/runtime/stapl_assert.hpp>
#include <stapl/utility/empty_class.hpp>

#include <stapl/paragraph/edge_container/edge_entry_base.hpp>

namespace stapl {

template<typename T>
struct lazy_edge_reference;

template<typename T>
struct pg_lazy_edge_reference;

template<typename ProxyContainer>
class aggregated_edge_subview;


namespace paragraph_impl {

class task_graph;

} // namespace paragraph_impl


namespace detail {

class edge_entry_base;

template<typename T>
class edge_entry;


//////////////////////////////////////////////////////////////////////
/// @brief Class containing everything needed to callback to PARAGRAPH
/// outside its communication group and notify it that a task has
/// completed.
//////////////////////////////////////////////////////////////////////
struct remote_task_metadata
{
  rmi_handle::reference m_handle_ref;
  unsigned int          m_lid;
  size_t                m_task_id;

  remote_task_metadata(rmi_handle::reference handle_ref,
                       unsigned int lid,
                       size_t task_id)
    : m_handle_ref(std::move(handle_ref)),
      m_lid(lid),
      m_task_id(task_id)
  { }
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Callback to PARAGRAPH used by tasks to inform it of
///   task completion, migration requests, etc.
/// @ingroup pgTasks
///
/// Tasks can be migrated to execute at locations outside where the associated
/// PARAGRAPH is defined.  This class generalizes task access to the PARAGRAPH
/// to support such cases.
///
/// @note Don't be fancy and use m_lid invalid comparison to save
///   allocation of boolean m_b_valid.  The comparison takes longer.
//////////////////////////////////////////////////////////////////////
class tg_callback
{
private:
  const bool m_b_valid;

  union {
    paragraph_impl::task_graph* const  m_tg;
    const detail::remote_task_metadata m_metadata;
  };

public:
  tg_callback(paragraph_impl::task_graph* tg_ptr)
    : m_b_valid(false), m_tg(tg_ptr)
  { }

  tg_callback(rmi_handle::reference handle_ref,
              unsigned int lid, size_t task_id)
    : m_b_valid(true), m_metadata(handle_ref, lid, task_id)
  { }

  paragraph_impl::task_graph& tg(void) const
  { return *m_tg; }

  bool valid(void) const
  { return m_b_valid; }

  rmi_handle::reference handle(void) const
  { return m_metadata.m_handle_ref; }

  unsigned int get_location_id(void) const
  { return m_metadata.m_lid; }

  size_t task_id(void) const
  { return m_metadata.m_task_id; };
}; // class tg_callback


//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to conditionally call release on a task's
///  view members if they are a @ref lazy_edge_reference. This functor
///  is for use in ephemeral paragraphs and few call sites where
///  lazy references are transiently created.
//////////////////////////////////////////////////////////////////////
class lazy_ref_release_func
{
private:
  tg_callback const& m_cb;

public:
  lazy_ref_release_func(tg_callback const& cb)
    : m_cb(cb)
  { }

  using result_type = void;

  template<typename T>
  void operator()(T& t) const
  { }

  template<typename T>
  void operator()(lazy_edge_reference<T>& t) const
  { t.release(m_cb); }

  template<typename T>
  void operator()(pg_lazy_edge_reference<T>& t) const
  { t.release(m_cb); }

  template<typename T>
  void operator()(aggregated_edge_subview<T>& t) const
  { t.release(m_cb); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to conditionally call release on a task's
///  view members if they are a @ref lazy_edge_reference. This functor
///  is for use in persistent paragraphs.
//////////////////////////////////////////////////////////////////////
struct lazy_ref_release_persistent_func
{
  using result_type = void;

  template<typename T>
  void operator()(T& t) const
  { }

  template<typename T>
  void operator()(lazy_edge_reference<T>& t) const
  { t.release(); }

  template<typename T>
  void operator()(pg_lazy_edge_reference<T>& t) const
  { t.release(); }

  template<typename T>
  void operator()(aggregated_edge_subview<T>& t) const
  { t.release(); }
};


namespace paragraph_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to compute storage type for lhs side of workfunction
///   invocation in @p Task::operator().  Allows void and nonvoid types to be
///   handled uniformly to avoid a complete specialization the @p Task class
///   template.
/// @ingroup pgTasks
///
/// @tparam Result The return type of the workfunction.
///
/// Primary template is for non void returning workfunction.
/// Reflect @p Result.
//////////////////////////////////////////////////////////////////////
template<typename Result>
struct result_storage_mf
{
  using type = Result;
};


//////////////////////////////////////////////////////////////////////
/// @brief Class specialization for workfunction returning void.
///   Reflect @p empty_class.
/// @ingroup pgTasks
//////////////////////////////////////////////////////////////////////
template<>
struct result_storage_mf<void>
{
  using type = empty_class;
};


//////////////////////////////////////////////////////////////////////
/// @brief Encapsulates all information needed to execute a task. Constructed
///   by the PARAGRAPH and passed to executor when all dependences have been
///   satisfied.
/// @ingroup pgTasks
//
/// This base class is type erased with virtual dispatch to the derived
/// classes' function operator, allowing any PARAGRAPH to support arbitrary
/// workfunction and view parameters.
///
/// @todo destructor for ephemeral_pg deletes factory_task, through task_base*
/// otherwise no need for this to be virtual,  since we never destruct an object
/// through this base class.  (Task::operator() does the deletion). Guard
/// or refactor appropriately to avoid needless virtual function dispatch.
///
/// @todo  inject an intermediate base class between task_base and task
/// (factory_task_base) so that finished and reinvoke can be pure virtual there
/// and not a part of @p task_base.  (can cast intermediate in PARAGRAPH).
//////////////////////////////////////////////////////////////////////
struct task_base
  : public runnable_base
{
private:
  detail::edge_entry_base* const m_edge_entry_ptr;

public:
  ~task_base() override = default;

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Extract the has successors boolean trait from the bit field
  ///   and return.
  ///
  /// @return True if this task has successors in the PARAGRAPH.
  //////////////////////////////////////////////////////////////////////
  bool has_succs() const
  {
    return m_edge_entry_ptr != nullptr;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Notify the PARAGRAPH that the task has finished execution
  /// and provide the return value to trigger data flow.
  /// Signature for void return values.
  ///
  /// @param ignore Parameter passed in place of void for genericity.
  ///   Unused.
  ///
  /// Called from derived class function operator implementations.
  //////////////////////////////////////////////////////////////////////
  void processed(empty_class&& ignore, bool b_has_succs,
                 tg_callback const&) const;


  //////////////////////////////////////////////////////////////////////
  /// @brief Notify the PARAGRAPH that the task has finished execution
  /// and provide the return value to trigger data flow.
  /// Signature for non void return values.
  ///
  /// @param val The return value of the workfunction invocation.
  ///
  /// Called from derived class function operator implementations.
  //////////////////////////////////////////////////////////////////////
  template<typename Return>
  void processed(Return&& val, bool b_has_succs, tg_callback const&) const;


  /// @brief the type returned by the function operator with status information
  /// for the executor.
  using result_type = runnable_base::status_type;

public:
  task_base(detail::edge_entry_base* edge_entry_ptr)
    : m_edge_entry_ptr(edge_entry_ptr)
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Extract task identifier from bit field and return it.
  //////////////////////////////////////////////////////////////////////
  std::size_t task_id() const
  {
    return m_edge_entry_ptr ? m_edge_entry_ptr->tid() : 0;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Use presence/lack of intra-PARAGRAPH successors to inform
  ///  the execution in which task communication group this task's traffic
  ///  should be placed.
  /// @param cb Callback mechanism to PARAGRAPH passed to task by
  ///   its executor.
  //////////////////////////////////////////////////////////////////////
  p_object const& comm_p_object(tg_callback const& cb) const;


  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke the task.  Called by the executor.  Implemented in
  ///   derived class that knows the workfunction and view types.
  /// @param cb Callback mechanism to PARAGRAPH passed to task by
  ///   its executor.
  //////////////////////////////////////////////////////////////////////
  virtual result_type operator()(tg_callback const& cb) = 0;


  //////////////////////////////////////////////////////////////////////
  /// @brief Return @c true if task can be migrated.
  ///
  /// This default implementation is used by the factory task, where migration
  /// is not allowed.
  //////////////////////////////////////////////////////////////////////
  virtual bool migratable(void) const noexcept
  {
    return false;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return true if task has completed and does not need reinvocation.
  /// This default implementation is used by the non factory tasks, where the
  /// method should never be called.
  //////////////////////////////////////////////////////////////////////
  virtual bool finished(void) const
  {
    abort("called task_base::finished()");

    return true;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Reinvoke the task if it has not completed.
  /// This default implementation is used by the non factory tasks, where the
  /// method should never be called.
  ///
  /// @returns @c true if it needs to be reinvoked again, otherwise @c false.
  //////////////////////////////////////////////////////////////////////
  virtual bool reinvoke(tg_callback const&)
  {
    abort("called task_base::reinvoke");
    return false;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Pure virtual method implemented by derived classes to report
  /// whether or not they are instance of the class template @p factory_task.
  ///
  /// Used by the runtime executor and scheduler to specialize scheduling
  /// (i.e., execute once immediately upon being made runnable).
  //////////////////////////////////////////////////////////////////////
  virtual bool is_factory_task(void) const noexcept = 0;

  //////////////////////////////////////////////////////////////////////
  /// @brief Allows detection of derived class acting as holder for nested
  /// paragraph.  Used @ref paragraph::operator() to detect starting tasks
  /// which are actually nested paragraphs to avoid passing them to the
  /// executor.
  //////////////////////////////////////////////////////////////////////
  virtual bool is_nested_pg_task(void) const noexcept
  { return false; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Intermediate class in @p task_base / @p Task inheritance
///   relationship which encodes the scheduler metadata type so that a virtual
///   function signature can be defined and polymorphically invoked by the
///   scheduler (who doesn't know concrete type of @p Task) to initiate task
///   migration.
/// @ingroup pgTasks
///
/// @tparam SchedInfo The type of per task scheduler metadata, as defined by
///   the scheduler type parameter of the PARAGRAPH.
///
/// The scheduler framework knows the SchedInfo type and can perform a
/// downcast of the @p task_base to this base, allowing it to invoke migration.
///
/// @todo @p migrate should probably be pure virtual.
/// @todo Class probably doesn't need to be in the class hierarchy of
///   @ref factory_task.
//////////////////////////////////////////////////////////////////////
template<typename SchedulerEntry>
class task_base_intermediate
  : public task_base,
    public SchedulerEntry
{
public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor used by factory tasks.
  //////////////////////////////////////////////////////////////////////
  task_base_intermediate(void)
    : task_base(nullptr),
      SchedulerEntry(typename SchedulerEntry::sched_info_type())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor used by standard (i.e., non factory) tasks.
  //////////////////////////////////////////////////////////////////////
  template<typename SchedulerInfoParam>
  task_base_intermediate(detail::edge_entry_base* edge_entry_ptr,
                         SchedulerInfoParam&& sched_info)
    : task_base(edge_entry_ptr),
      SchedulerEntry(std::forward<SchedulerInfoParam>(sched_info))
  { }

  ~task_base_intermediate() override = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Method overridden in @p Task to request migration of this task by
  ///   @ref task_graph.
  /// @param dest_loc The location the task should be migrated to.
  /// @param sched_info The scheduler metadata for this task.
  /// @param cb Callback wrapper to the associated PARAGRAPH.
  //////////////////////////////////////////////////////////////////////
  virtual void migrate(size_t dest_loc, tg_callback const& cb)
  {
    abort("called task_base::migrate(size_t)");
  }
};


template <typename SchedulerEntry,
          typename EnableMigration,
          typename EnablePersistence,
          typename WF,
          typename ViewSet>
class Task;


template <typename Factory, typename ViewSet,
          typename SchedulerEntry, typename TGV>
class factory_task;

template<typename SchedulerEntry, typename PG>
class nested_pg_task;

} // namespace paragraph_impl

} // namespace stapl

#endif
