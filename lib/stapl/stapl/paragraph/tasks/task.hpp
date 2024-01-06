/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_TASK_HPP
#define STAPL_PARAGRAPH_TASK_HPP

#include <type_traits>

#include <stapl/utility/integer_sequence.hpp>
#include <stapl/paragraph/tasks/task.h>
#include <stapl/paragraph/paragraph_impl.hpp>
#include <stapl/paragraph/paragraph_view.hpp>

namespace stapl {

namespace paragraph_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Implementation class of @ref pre_execute_func.  Uses partial
/// specialization to call method in object if and only if it is a view.
/// @ingroup pgTasks
//////////////////////////////////////////////////////////////////////
template<typename T, bool = is_view<T>::value>
struct pre_execute_impl
{
  static void apply(T& t)
  { t.pre_execute(); }
};


template<typename T>
struct pre_execute_impl<T, false>
{
  static void apply(T& t)
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Implementation class of @ref post_execute_func.  Uses partial
/// specialization to call method in object if and only if it is a view.
/// @ingroup pgTasks
//////////////////////////////////////////////////////////////////////
template<typename T, bool = is_view<T>::value>
struct post_execute_impl
{
  static void apply(T& t)
  { t.post_execute(); }
};


template<typename T>
struct post_execute_impl<T, false>
{
  static void apply(T& t)
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function object used by @ref Task to invoke @p pre_execute on each
///   element of the viewset unless it is a proxy or @ref paragraph_view.
/// @ingroup pgTasks
//////////////////////////////////////////////////////////////////////
struct pre_execute_func
{
  typedef void result_type;

  template<typename T>
  void operator()(T& t) const
  { pre_execute_impl<T>::apply(t); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function object used by @p Task to invoke @p post_execute on each
///   element of the viewset unless it is a proxy or @ref paragraph_view.
/// @ingroup pgTasks
//////////////////////////////////////////////////////////////////////
struct post_execute_func
{
  typedef void result_type;

  template<typename T>
  void operator()(T& t) const
  { post_execute_impl<T>::apply(t); }
};


inline
p_object const& task_base::comm_p_object(tg_callback const& cb) const
{
  stapl_assert(!cb.valid(), "expected invalid tg_callback");

  return this->has_succs() ?
    cb.tg().succs_p_object() : cb.tg().no_succs_p_object();
}


inline
void task_base::processed(empty_class&&, bool b_has_succs,
                          tg_callback const& cb) const
{
  if (!cb.valid())
  {
    if (b_has_succs)
      this->m_edge_entry_ptr->set_value(cb.tg().edges());

    cb.tg().processed_local();

    return;
  }

  // else
  using mem_fun_t = void (task_graph::*)(std::size_t, bool);

  const mem_fun_t mem_fun = &task_graph::processed_remote_void;

  async_rmi(cb.get_location_id(), cb.handle(),
            mem_fun, cb.task_id(), b_has_succs);
}


//////////////////////////////////////////////////////////////////////
/// @brief Type metafunction strips @ref immutable_shared wrapper from
/// reflected type if it exists.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct strip_wrapper
{ typedef T type; };


template<typename T>
struct strip_wrapper<immutable_shared<T>>
{ typedef T type; };


template<typename Return>
inline
void task_base::processed(Return&& val, bool b_has_succs,
                          tg_callback const& cb) const
{
  if (!cb.valid())
  {
    typedef typename compute_df_edge_type<
      typename strip_wrapper<typename std::decay<Return>::type>::type
    >::type edge_type;

    if (b_has_succs)
      static_cast<detail::edge_entry<edge_type>*>
        (this->m_edge_entry_ptr)->template
          set_value<detail::df_identity<
            typename df_stored_type<edge_type>::type>>
              (std::forward<Return>(val), cb.tg().edges(), 0, true);

    cb.tg().processed_local();

    return;
  }

  // else
  typedef void (task_graph::*mem_fun_t)
    (std::size_t, bool,
     typename strip_wrapper<
       typename std::decay<Return>::type>::type const&);

  const mem_fun_t mem_fun = &task_graph::processed_remote;

  async_rmi(cb.get_location_id(), cb.handle(), mem_fun, cb.task_id(),
            b_has_succs, val);
}

} // namespace paragraph_impl


namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction that inserts a const qualification on the
///   workfunction if persistency mode in the PARAGRAPH is enabled.
/// @ingroup pgTasks
///
/// @param WF The initial workfunction type.
/// @param b_persistent Boolean that is true if PARAGRAPH has persistency
///   enabled.
///
/// Primary template matches b_persistent == false case.  Reflect @p WF.
///
/// Enforcing const qualification for persistent tasks enforces that
/// the workfunction is free of side-effects (assuming workfunction writer
/// doesn't purposely circumvent the type system), an necessary trait for
/// correct reinvocation that avoid regeneration of all tasks by the factory.
//////////////////////////////////////////////////////////////////////
template<typename WF, bool b_persistent>
struct compute_invoked_wf
{
  typedef WF type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when b_persistent == true.
///   Reflect @p WF with const qualification added (if not already present).
/// @ingroup pgTasks
//////////////////////////////////////////////////////////////////////
template<typename WF>
struct compute_invoked_wf<WF, true>
  : public std::add_const<WF>
{ };

} // namespace detail


namespace paragraph_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Concrete implementation of abstract class @p task, used to invoke
///   user workfunction with views that were defined as task via an @p add_task
///   method call on the @ref paragraph_view.
/// @ingroup pgTasks
///
/// @tparam SchedulerEntry The entry type required to store this task and
///  the associated scheduler info metadata in the Scheduler.
/// @tparam EnableMigration Boolean type parameter denoting whether migration
///   is enabled for the associated PARAGRAPH.
/// @tparam EnablePersistence Boolean type parameter denoting whether
///    persistency is enabled for the associated PARAGRAPH.
/// @tparam WF The workfunction type.
/// @tparam ViewSet The type of the set of views passed to the workfunction.
//////////////////////////////////////////////////////////////////////
template<typename SchedulerEntry,
         typename EnableMigration,
         typename EnablePersistence,
         typename WF,
         typename ViewSet>
class Task final
    : public task_base_intermediate<SchedulerEntry>,
      private WF,
      private ViewSet
{
private:
  friend class task_graph; // for access to viewset for migration

  typedef task_base_intermediate<SchedulerEntry>     task_base_intermediate_t;
  typedef typename ViewSet::view_set_type            viewset_t;

  typedef typename detail::compute_invoked_wf
    <WF, EnablePersistence::value>::type             invoke_wf_t;

  /// @brief Type of helper functor that creates comma separated parameter
  /// list from the tuple representation of the viewset.
  typedef wf_invoke<invoke_wf_t, viewset_t>          invoker_t;

  typedef typename invoker_t::result_type            result_t;

  /// @brief Compute storage type for return of workfunction invocation.
  typedef typename result_storage_mf<result_t>::type storage_t;


  typedef typename task_base_intermediate_t::
                                         result_type result_type;

  ViewSet& viewset(void)
  {
    return static_cast<ViewSet&>(*this);
  }

public:
  template<typename SchedInfoParam, typename WFParam, typename ...ViewParams>
  Task(tg_callback const& cb,
       detail::edge_local_notifier_base* notifier_ptr,
       detail::edge_entry_base* edge_entry_ptr,
       SchedInfoParam&& si, WFParam&& wf,
       ViewParams&&... views)
    : task_base_intermediate_t(edge_entry_ptr,
                               std::forward<SchedInfoParam>(si)),
      WF(std::forward<WFParam>(wf)),
      ViewSet(cb, notifier_ptr, std::forward<ViewParams>(views)...)
  { }

  STAPL_USE_MANAGED_ALLOC(Task)

  //////////////////////////////////////////////////////////////////////
  /// @brief Used internally to generically invoke workfunctions with
  ///   nonvoid return types.
  ///
  /// Unused parameter is type tag dispatch from @p operator().
  //////////////////////////////////////////////////////////////////////
  storage_t invoke(std::false_type)
  {
    return invoker_t()(static_cast<WF&>(*this), this->viewset().views());
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Used internally to generically invoke workfunctions with a void
  ///   return type.
  ///
  /// Unused parameter is type tag dispatch from @p operator().
  //////////////////////////////////////////////////////////////////////
  empty_class invoke(std::true_type)
  {
    invoker_t()(static_cast<WF&>(*this), this->viewset().views());
    return empty_class();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Invokes workfunction with the specified view set.
  //////////////////////////////////////////////////////////////////////
  result_type operator()(tg_callback const& cb) final
  {
    vs_map(pre_execute_func(), this->viewset().views());

    storage_t tmp =
      invoke(typename std::is_same<empty_class, storage_t>::type());

    vs_map(post_execute_func(), this->viewset().views());

    const bool b_has_succs = this->has_succs();

    if (b_has_succs || cb.valid())
      rmi_fence();

    this->release(EnablePersistence(), cb);

    this->processed(std::move(tmp), b_has_succs, cb);

    // If we're persistent, defer destruction and leave it to the paragraph's
    // destructor.  If we're ephemeral, delete it now.
    this->cleanup(EnablePersistence(), cb);

    return task_base_intermediate_t::Finished;
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Used internally to disable migration when the PARAGRAPH does not
  /// support this feature.
  ///
  /// Unused parameter is type tag dispatch from @p operator().
  //////////////////////////////////////////////////////////////////////
  void migrate_impl(std::size_t, std::false_type, tg_callback const&)
  {
    stapl_assert(false, "scheduler has disabled migration");
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Used internally to enable migration when the PARAGRAPH supports
  /// this feature.
  ///
  /// @param dest The new execution location for the task.
  /// @param sched_info The associated per task scheduling metadata.
  /// Unused parameter is type tag dispatch from @p operator().
  //////////////////////////////////////////////////////////////////////
  void migrate_impl(std::size_t dest, std::true_type, tg_callback const& cb)
  {
    stapl_assert(!cb.valid(), "tg_callback pointer isn't initialized");

    // ask the task_graph to migrate this task
    cb.tg().migrate_task(
      this, dest,
      make_index_sequence<tuple_size<viewset_t>::value>());

    this->release(EnablePersistence(), cb, true);

    delete this;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used internally to enable self deletion when called from a
  ///  ephemeral PARAGRAPH.
  ///
  /// Unused parameter is type tag dispatch from @p operator().
  //////////////////////////////////////////////////////////////////////
  void release(std::false_type, tg_callback const& cb, bool migrate = false)
  {
    this->viewset().release(cb, migrate);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used internally to avoid self deletion when called from a
  ///  persistent PARAGRAPH.
  ///
  /// Unused parameter is type tag dispatch from @p operator().
  //////////////////////////////////////////////////////////////////////
  void release(std::true_type, tg_callback const&)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used internally to enable self deletion when called from a
  ///  ephemeral PARAGRAPH.
  ///
  /// Unused parameter is type tag dispatch from @p operator().
  //////////////////////////////////////////////////////////////////////
  void cleanup(std::false_type, tg_callback const& cb)
  {
    delete this;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used internally to avoid self deletion when called from a
  ///  persistent PARAGRAPH.
  ///
  /// Unused parameter is type tag dispatch from @p operator().
  //////////////////////////////////////////////////////////////////////
  void cleanup(std::true_type, tg_callback const&)
  { }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Return @c true if this task can be migrated.
  ///
  /// In this context, this is true if migration was enabled in the PARAGRAPH.
  //////////////////////////////////////////////////////////////////////
  bool migratable(void) const noexcept final
  {
    return EnableMigration();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Initiates migration of this task to a new execution location
  /// @p dest along with associated scheduling metadata @p sched_info.
  ///
  /// Called by schedulers if they wish to change the initial PARAGRAPH task
  /// distribution induced by their task placement policy.
  //////////////////////////////////////////////////////////////////////
  void migrate(std::size_t dest, tg_callback const& cb) final
  {
    migrate_impl(dest, EnableMigration(), cb);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Implementation of pure virtual method defined in abstract
  ///   class @p task.
  /// @sa task::is_factory_task
  //////////////////////////////////////////////////////////////////////
  bool is_factory_task(void) const noexcept final
  {
    return false;
  }


  ~Task() final
  {
    if (EnablePersistence::value)
      this->viewset().release();
  }
}; // class Task (primary template / general case)


//////////////////////////////////////////////////////////////////////
/// @brief Implementation base class for @ref Task class template
/// specializations involving identity tasks.  Inspects the incoming
/// edge and attempts to move / share the associated value object
/// instead of copying.
///
/// @todo Conditionally avoid optimization for small basic types
/// (i.e., fundamental types) to avoid needless overhead.
//////////////////////////////////////////////////////////////////////
template<typename SchedulerEntry,
         typename EnableMigration,
         typename EnablePersistence,
         typename ViewSet>
class identity_task_impl
  : public task_base_intermediate<SchedulerEntry>,
    private ViewSet
{
private:
  friend class task_graph; // for access to viewset for migration

  using task_base_intermediate_t = task_base_intermediate<SchedulerEntry>;

  using typename task_base_intermediate_t::result_type;

  void cleanup(std::false_type)
  { delete this; }

  void cleanup(std::true_type)
  { }

public:
  template<typename SchedInfoParam, typename WFParam, typename ViewParam>
  identity_task_impl(tg_callback const& cb,
                     detail::edge_local_notifier_base* notifier_ptr,
                     detail::edge_entry_base* edge_entry_ptr,
                     SchedInfoParam&& si, WFParam const&,
                     ViewParam&& view)
    : task_base_intermediate_t(
        edge_entry_ptr, std::forward<SchedInfoParam>(si)),
      ViewSet(cb, notifier_ptr, std::forward<ViewParam>(view))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invokes workfunction with the specified view set.
  //////////////////////////////////////////////////////////////////////
  result_type operator()(tg_callback const& cb) final
  {
    const bool b_has_succs = this->has_succs();

    auto& in_edge = get<0>(static_cast<ViewSet&>(*this).views());

    pre_execute_func()(in_edge);

    if (in_edge.is_direct_storage())
    {
      if (in_edge.stealable())
      {
        auto val = in_edge.steal();
        static_cast<ViewSet&>(*this).release(cb);
        this->processed(std::move(val), b_has_succs, cb);
      }
      else
      {
        auto val = in_edge.get_storage_ref();
        static_cast<ViewSet&>(*this).release(cb);
        this->processed(std::move(val), b_has_succs, cb);
      }
    }
    else
    {
      auto wrapper = in_edge.get_shared_wrapper();
      static_cast<ViewSet&>(*this).release(cb);
      this->processed(std::move(wrapper), b_has_succs, cb);
    }

    post_execute_func()(in_edge);

    // If we're persistent, defer destruction and leave it to the paragraph's
    // destructor.  If we're ephemeral, delete it now.
    this->cleanup(EnablePersistence());

    return task_base_intermediate_t::Finished;
  }

  bool migratable(void) const noexcept final
  { return EnableMigration(); }

  void migrate(std::size_t dest, tg_callback const& cb) final
  { abort("Tried to migrate an identity task"); }

  bool is_factory_task(void) const noexcept final
  { return false; }

  ~identity_task_impl() override
  {
    if (EnablePersistence::value)
      static_cast<ViewSet&>(*this).release();
  }

  STAPL_USE_MANAGED_ALLOC(identity_task_impl)
}; // class identity_task_impl


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for @ref identity_op that applies identity
/// task squashing optimization implemented in @ref identity_task_impl.
//////////////////////////////////////////////////////////////////////
template<typename SchedulerEntry,
         typename EnableMigration, typename EnablePersistence,
         template<typename...> class Storage, typename T>
class Task<SchedulerEntry, EnableMigration, EnablePersistence,
           identity_op, Storage<lazy_edge_reference<T>>> final
  : public identity_task_impl<
      SchedulerEntry, EnableMigration, EnablePersistence,
      Storage<lazy_edge_reference<T>>>
{
private:
  using base_type =
    identity_task_impl<
      SchedulerEntry, EnableMigration, EnablePersistence,
      Storage<lazy_edge_reference<T>>>;

public:
  using base_type::base_type;
}; // class Task<..., identity_op, ...>


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for @ref identity that applies identity
/// task squashing optimization implemented in @ref identity_task_impl.
//////////////////////////////////////////////////////////////////////
template<typename SchedulerEntry,
         typename EnableMigration, typename EnablePersistence,
         template<typename...> class Storage, typename T, typename Q>
class Task<SchedulerEntry, EnableMigration, EnablePersistence,
           identity<Q>, Storage<lazy_edge_reference<T>>> final
  : public identity_task_impl<
      SchedulerEntry, EnableMigration, EnablePersistence,
      Storage<lazy_edge_reference<T>>>
{
private:
  using base_type =
    identity_task_impl<
      SchedulerEntry, EnableMigration, EnablePersistence,
      Storage<lazy_edge_reference<T>>>;

public:
  using base_type::base_type;
}; // class Task<..., identity<Q>, ...>


//////////////////////////////////////////////////////////////////////
/// @brief Metafunction that invokes member @p finished on the @p factory
///   if it inherits from @ref incremental_wf, denoting that it
///   implements the incremental workfunction concept.
/// @ingroup pgTasks
///
/// Primary template matches incremental case.  Invoke @p finished
///   on the factory.
//////////////////////////////////////////////////////////////////////
template<typename Factory,
         bool = std::is_base_of<incremental_wf, Factory>::value>
struct check_finished
{
  static bool apply(Factory const& factory)
  {
    return factory.finished();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization matching case when @p Factory conforms to the
///  incremental workfunction concept.
/// @ingroup pgTasks
///
/// Return true, no invocation is required.
//////////////////////////////////////////////////////////////////////
template<typename Factory>
struct check_finished<Factory, false>
{
  static bool apply(Factory const&)
  {
    return true;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Concrete implementation of abstract class @ref task_base, used to
///   invoke the factory (the initial task executed for any PARAGRAPH which
///   populates the graph with additional tasks) from the PARAGRAPH's executor.
/// @ingroup pgTasks
/// @tparam Factory Type type of the factory passed to the PARAGRAPH at
///   initialization.
/// @tparam ViewSet The type of the post coarsening set of PARAGRAPH input
///   views to be passed to the factory.
/// @tparam PGV The type of the @ref paragraph_view for this PARAGRAPH.
///   No need for member, can be trivially default constructed.
//////////////////////////////////////////////////////////////////////
template<typename Factory, typename ViewSet,
         typename SchedulerEntry, typename PGV>
class factory_task final
  : public task_base_intermediate<SchedulerEntry>
{
private:
  using result_type = typename task_base::result_type;

   /// @brief The PARAGRAPH's factory object.
  Factory m_factory;

  /// @brief The set of views to be passed to factory invocations.
  /// The post-coarsening input views for this PARAGRAPH.
  ViewSet m_vs;

public:
  factory_task(Factory factory, ViewSet&& vs)
    : m_factory(std::move(factory)),
      m_vs(std::move(vs))
  { }

  Factory const& factory(void) const
  {
    return m_factory;
  }

  ViewSet& views(void)
  {
    return m_vs;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invokes factory to populate PARAGRAPH with tasks.
  //////////////////////////////////////////////////////////////////////
  result_type operator()(tg_callback const& cb) final
  {
    stapl_assert(!cb.valid(), "tg_callback pointer isn't initialized");

    // For now PGV is stateless and so just its default construction suffices.
    // In future if PGV has state then class factory_task hold tgv as a member.
    wf_invoke_void<Factory, ViewSet>()(m_factory, m_vs, PGV(cb.tg()));

    this->processed(empty_class(), false, cb);

    return task_base::Finished;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return true if the factory has created all tasks it wishes to
  ///  insert into the PARAGRAPH.
  //////////////////////////////////////////////////////////////////////
  bool finished(void) const final
  {
    return check_finished<Factory>::apply(m_factory);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used by PARAGRAPH for subsequent calls to the factory to create
  ///   any additional tasks.
  ///
  /// Guards reinvocation with @p finished == false to avoid calling a factory
  /// with no further tasks to create.
  ///
  /// @returns @c true if it needs to be reinvoked again, otherwise @c false.
  //////////////////////////////////////////////////////////////////////
  bool reinvoke(tg_callback const& cb) final
  {
    if (this->finished())
      return false;

    stapl_assert(!cb.valid(), "tg_callback pointer isn't initialized");

    gang g(cb.tg());

    wf_invoke_void<Factory, ViewSet>()(m_factory, m_vs, PGV(cb.tg()));

    return !this->finished();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Implementation of pure virtual method defined in abstract
  ///   class @p task.
  /// @sa task::is_factory_task
  //////////////////////////////////////////////////////////////////////
  bool is_factory_task(void) const noexcept final
  {
    return true;
  }
}; // class factory_task


//////////////////////////////////////////////////////////////////////
/// @brief Task wrapper for nested @ref paragraph.  Used to reinvoke
/// the paragraph in from a persistent parent paragraph.
//////////////////////////////////////////////////////////////////////
template<typename SchedulerEntry, typename PG>
class nested_pg_task final
  : public task_base_intermediate<SchedulerEntry>
{
private:
  rmi_handle::reference m_handle;

public:
  using task_base_intermediate_t = task_base_intermediate<SchedulerEntry>;

  using result_type = typename task_base_intermediate_t::result_type;

  STAPL_USE_MANAGED_ALLOC(nested_pg_task)

  template<typename SchedInfoParam>
  nested_pg_task(rmi_handle::reference handle, SchedInfoParam&& si)
    : task_base_intermediate_t(
        nullptr, std::forward<SchedInfoParam>(si)),
      m_handle(std::move(handle))
  { }

  result_type operator()(tg_callback const&) final
  {
    typedef typename PG::result_type (PG::* mem_fun_t)(int, bool, bool, bool);

    constexpr mem_fun_t mem_fun = &PG::operator();

    unordered::async_rmi(all_locations, m_handle, mem_fun,
                         0, false, false, true);

    return task_base_intermediate_t::Finished;
  }

  bool is_factory_task(void) const noexcept final
  { return false; }

  bool is_nested_pg_task(void) const noexcept final
  { return true; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Triggers asynchronous deletion of paragraph this task
  /// encapsulates.
  //////////////////////////////////////////////////////////////////////
  ~nested_pg_task()
  {
    typedef void (PG::* mem_fun_t)(void);

    constexpr mem_fun_t mem_fun = &PG::try_os_delete;

    unordered::async_rmi(all_locations, m_handle, mem_fun);
  }
}; // class nested_pg_task

} // namespace paragraph_impl

} // namespace stapl

#endif
