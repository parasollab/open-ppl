/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_PARAGRAPH_IMPL_HPP
#define STAPL_PARAGRAPH_PARAGRAPH_IMPL_HPP

#include <functional>
#include <vector>
#include <utility>
#include <memory>
#include <boost/bind.hpp>

#include <stapl/utility/down_cast.hpp>
#include <stapl/utility/tuple.hpp>
#include <stapl/utility/integer_sequence.hpp>

#include <stapl/runtime/executor/executor.hpp>
#include <stapl/runtime/executor/scheduler/sched.hpp>

#include <stapl/paragraph/result_container_base.hpp>
#include <stapl/paragraph/tasks/task.h>
#include <stapl/paragraph/task_migration.hpp>
#include <stapl/paragraph/cross_tg_notifier.hpp>
#include <stapl/paragraph/edge_container/edge_local_notifier.hpp>
#include <stapl/paragraph/edge_container/edge_container.h>
#include <stapl/paragraph/edge_container/views/edge_view_fwd.h>

#include <stapl/skeletons/spans/per_location.hpp>

#include <stapl/skeletons/utility/utility.hpp>

#include <stapl/views/metadata/partitioned_mix_view_fwd.hpp>
#include <stapl/views/metadata/localize_object.hpp>
#include <stapl/views/view_packing.hpp>

#include <stapl/paragraph/tasks/task_creation.h>


namespace stapl {


//////////////////////////////////////////////////////////////////////
/// @brief Allow nested paragraphs to send message to location in
/// parent paragraph with same affinity if parent location has not been
/// constructed yet.
//////////////////////////////////////////////////////////////////////
void add_pg_message(rmi_handle::reference h,
                    std::function<void (paragraph_impl::task_graph&)> msg);


//////////////////////////////////////////////////////////////////////
/// @brief Allow nested paragraph edge_container to send message to
/// location in parent paragraph input port (edge_container) with same
/// affinity if parent location has not been constructed yet.
//////////////////////////////////////////////////////////////////////
void add_ec_message(rmi_handle::reference h,
                    std::function<void (edge_container&)> msg);

namespace detail {

template<typename T>
struct result_container;

} // namespace detail


template<typename T, typename A>
class proxy;

template<typename View>
struct ptr_wrapper;

template<typename T, typename... OptionalFilter>
class aggregated_edge_view;

class edge_container;


namespace paragraph_impl {

BOOST_MPL_HAS_XXX_TRAIT_DEF(scheduler_type)
BOOST_MPL_HAS_XXX_TRAIT_DEF(task_placement_all_local)
BOOST_MPL_HAS_XXX_TRAIT_DEF(span_type)


template <typename Factory, typename ViewSet>
struct need_result_view
  : skeletons::is_output_scalar<Factory, ViewSet>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Basic generalization of a directory for keys referring to a
/// distributed object. Specific use here is to forward consumption requests
/// from one nested paragraph to another.
///
/// @note Could avoid unregistration traffic for a very regular dataflow
/// pattern where exact number of message (setup_flow / num_succ + num pins
/// flowed per location are known.
///
/// @todo Unpopulate m_handles during unregister_key.  Maybe couple with
/// moving this metadata to the mapped location, instead of the registering
/// location.
//////////////////////////////////////////////////////////////////////
class nested_pg_directory
  : public directory<
      tuple<std::size_t, location_type>,
      use_default, use_default, use_default, true>
{
private:
  using base_t =
    directory<
      tuple<std::size_t, location_type>,
      use_default, use_default, use_default, true>;

public:
  using base_t::key_type;

  using mapped_type = rmi_handle::reference;

  std::unordered_map<
    key_type, mapped_type, stapl::hash<key_type>> m_handles;

  task_graph*                                     m_tg_ptr;

  template<typename TaskMapperParam>
  nested_pg_directory(task_graph* tg_ptr, TaskMapperParam&& task_mapper_param)
    : base_t([](tuple<std::size_t, location_type>) { return true; },
             detail::default_key_mapper<key_type>(),
             false, false),
      m_tg_ptr(tg_ptr)
   { }

  nested_pg_directory(nested_pg_directory const&)            = delete;
  nested_pg_directory& operator=(nested_pg_directory const&) = delete;

  ~nested_pg_directory()
  {
    // Wait for any incoming unregister key from nested paragraph locations.
    block_until([this]{ return this->m_registry.empty(); });
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Add information about the given nested paragraph into the
  /// metadata storage (called at the managing location for the key
  /// (i.e., task_id).  Then, register the key in the directory base class.
  ///
  /// @param key task id and location pair representing the registration.
  /// @param handle The rmi handle of the registration child paragraph.
  //////////////////////////////////////////////////////////////////////
  void add_pg(key_type key, rmi_handle::reference handle)
  {
    stapl_assert(m_handles.find(key) == m_handles.end(),
                 "Found producer in local cache");

    // Store information about the producer child paragraph on location
    // that called add_pg (since this is the location in the parent
    // sharing affinity with corresponding location in child.
    m_handles.insert(std::make_pair(key, handle));
    this->register_key(key);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Forward request to a producer nested paragraph that it flows
  ///   its return value to the consumer nested paragraph with
  ///   @ref rmi_handle::reference @p consumer.
  ///
  /// @tparam T The type of elements stored in the target producer's
  ///   output port.
  ///
  /// @param consumer_parent_location The location of the parent paragraph
  ///   colocated at the same affinity as the consumer paragraph that is
  ///   requesting dataflow.
  ///
  /// @param producer Task identifier of the producer PARAGRAPH in its
  ///   enclosing / parent PARAGRAPH.
  ///
  /// @param consumer Task identifier of the consumer PARAGRAPH that
  ///   is requesting dataflow.
  ///
  /// @param consumer_handle RMI handle of the consuming PARAGRAPH.
  ///
  /// @param consumer_location location in the consuming PARAGRAPH initiating
  ///   the point to point consumption request.
  ///
  /// @param port_filter Used to select which pins of the producer port
  ///   are forwarded to this consumer.
  ///
  /// @param value_filter Applied to each flowed pin's value, prior to
  ///   dataflow to the consumer.
  ///
  /// @param location_mapper maps producer pin to location in this consumer
  ///   where it should be flowed.
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename PortFilter, typename ValueFilter,
           typename LocationMapper>
  void setup_flow(location_type consumer_parent_location,
                  std::size_t producer,
                  std::size_t consumer,
                  rmi_handle::reference consumer_handle,
                  location_type consumer_location,
                  PortFilter port_filter,
                  ValueFilter value_filter,
                  LocationMapper location_mapper);

  //////////////////////////////////////////////////////////////////////
  /// @brief Forward request to a producer nested paragraph that it flows
  ///   its return value to its parent output port  using the specified
  ///   mapper class that remaps indices and filter dataflow if necessary.
  //////////////////////////////////////////////////////////////////////
  template<typename Mapper>
  void setup_parent_flow(std::size_t producer, Mapper mapper);
}; // class nested_pg_directory


//////////////////////////////////////////////////////////////////////
/// @brief Static functor with partial specialization to detect
///   a scheduler using task migration and provide it with a pointer
///   to the now initialized paragraph.
//////////////////////////////////////////////////////////////////////
template<typename Scheduler,
         bool=has_enable_migration<typename Scheduler::sched_info_type>::value>
struct initialize_migrating_scheduler
{
  static void apply(Scheduler&, task_graph&)
  { }
};


template<typename Scheduler>
struct initialize_migrating_scheduler<Scheduler, true>
{
  static void apply(Scheduler& scheduler, task_graph& tg)
  {
    scheduler.set_tg(tg);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief constexpr class used by callers of add_task to signify
/// that there are no explicit, signal only consumers of a given task.
//////////////////////////////////////////////////////////////////////
struct no_preds_t
{
  constexpr size_t size(void) const
  { return 0; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function called to setup signal flow on a list of predecessor
/// task_ids to the task using the given notifier.
//////////////////////////////////////////////////////////////////////
inline void
setup_signal(std::vector<std::size_t> const& pred_list,
             edge_container* edge_ct_ptr,
             detail::edge_local_notifier_base* notifier_ptr)
{
  for (size_t task_id : pred_list)
    edge_ct_ptr->setup_signal(task_id, notifier_ptr);
}


//////////////////////////////////////////////////////////////////////
/// @brief Signature for when @p no_preds_t object is passed to
/// @p add_task. In this case, do nothing.
//////////////////////////////////////////////////////////////////////
inline void
setup_signal(no_preds_t const&, edge_container*,
             detail::edge_local_notifier_base*)
{ }


//////////////////////////////////////////////////////////////////////
/// @brief Type metafunction to strip @ref pointer_wrapper from the type
/// if it exists.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct unwrap_pointer
  : public std::remove_pointer<T>
{ };


template<typename T, bool B>
struct unwrap_pointer<pointer_wrapper<T, B>>
{ using type = T; };


//////////////////////////////////////////////////////////////////////
/// @brief Used to distinguish one-sided nested and collective @ref paragraph
/// constructor calls.
//////////////////////////////////////////////////////////////////////
struct nested_tag
{ };

using std::size_t;
using detail::cross_tg_notifier;

//////////////////////////////////////////////////////////////////////
/// @struct compute_tid_mapper_type
/// @brief Extracts the key mapper type from the Factory class if it is defined.
/// @tparam Factory Factory class of a task_graph instance.
/// @tparam NumViews Number of views passed to the @ref paragraph.
/// @tparam Sequence index_sequence used for variadic expansion of views.
/// @tparam b Indicates if Factory defines task_id_mapper_type.
/// @ingroup paragraph
///
/// This default case for factories that don't specify the type.
//////////////////////////////////////////////////////////////////////
template<typename Factory,
         size_t NumViews,
         typename Sequence = make_index_sequence<NumViews>,
         bool b   = has_task_id_mapper_type<Factory>::value>
struct compute_tid_mapper_type
{
  template<typename ViewSet>
  static
  detail::default_key_mapper<size_t>
  apply(Factory const& f, ViewSet const&)
  {
    return detail::default_key_mapper<size_t>();
  }
};


//////////////////////////////////////////////////////////////////////
/// @struct compute_tid_mapper_type
/// @brief Extracts the key mapper type from the Factory class if it is defined.
/// @tparam Factory Factory class of a task_graph instance.
/// @tparam b Indicates if Factory defines task_id_mapper_type.
/// @ingroup paragraph
///
/// Factories that provide custom mapping of task ids to managing locations
/// reflect the task_id_mapper_type and provide a get_task_id_mapper method
/// that accepts the number of views the task_graph instance was given and
/// returns the mapper object.
//////////////////////////////////////////////////////////////////////
template<typename Factory, size_t NumViews, size_t ...Indices>
struct compute_tid_mapper_type<
  Factory, NumViews, index_sequence<Indices...>, true>
{
  template<typename ViewSet>
  static auto apply(Factory const& f, ViewSet const& vs)
  -> decltype(f.get_task_id_mapper(get<Indices>(vs)...))
  {
    return f.get_task_id_mapper(get<Indices>(vs)...);
  }
};


//////////////////////////////////////////////////////////////////////
/// @struct compute_df_edge_type
/// @brief Temporary metafunction used in task_graph::processed() to extract
///   proxies from proxy_holders they were wrapped in for serialization.
/// @ingroup paragraph
///
/// @todo Remove when usage in task_graph::processed is killed.
//////////////////////////////////////////////////////////////////////
template <typename T>
struct compute_df_edge_type
{
  typedef T type;
};


//////////////////////////////////////////////////////////////////////
/// @struct compute_df_edge_type
/// @brief Specialization to extract the proxy from proxy_holder.
/// @ingroup paragraph
///
/// @todo Remove when usage in task_graph::processed is killed.
//////////////////////////////////////////////////////////////////////
template<typename Proxy>
struct compute_df_edge_type<proxy_holder<Proxy>>
{
  typedef Proxy type;
};


//////////////////////////////////////////////////////////////////////
/// @struct is_deferred
/// @brief Default metafunction used in the identification of deferred
///   evaluation views and accessors in @ref input_edge_available_func.
/// @ingroup paragraph
//////////////////////////////////////////////////////////////////////
template<typename T, typename Q = void>
struct is_deferred
  : std::false_type
{ };


//////////////////////////////////////////////////////////////////////
/// @struct is_deferred
/// @brief Specialization to identify deferred evaluation views.
/// @ingroup paragraph
/// @see input_edge_available_func
//////////////////////////////////////////////////////////////////////
template<typename T>
struct is_deferred<T, typename T::deferred_evaluation_view_>
  : std::true_type
{ };


//////////////////////////////////////////////////////////////////////
/// @struct is_deferred
/// @brief Specialization to identify deferred evaluation accessors.
/// @ingroup paragraph
/// @see input_edge_available_func
//////////////////////////////////////////////////////////////////////
template<typename T>
struct is_deferred<T, typename T::deferred_evaluation_accessor_>
  : std::true_type
{ };


//////////////////////////////////////////////////////////////////////
/// @struct count_df_preds_func
/// @brief Functor used to identify deferred evaluation views and accessors
///   in the specification of a task.
/// @ingroup paragraph
/// @todo This count is actually resolvable at compile time.  If we want
/// to optimize the no df_preds path explicitly, use mpl::count.
//////////////////////////////////////////////////////////////////////
struct count_df_preds_func
{
  ////////////////////////////////////////////////////////////////////
  /// @brief default case for non-deferred evaluation parameters.
  ////////////////////////////////////////////////////////////////////
  template<typename Spec>
  static
  std::size_t
  apply(Spec const&)
  {
    return 0;
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Counts the specificiation of subviews backed by edge_containers.
  ///
  /// @todo Determine if this is deadcode.
  ////////////////////////////////////////////////////////////////////
  template<typename T>
  static
  std::size_t
  apply(std::pair<edge_view<T>*,
             typename edge_view<T>::index_type> const&)
  {
    return 1;
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Counts the specificiation of consumed results of preceeding tasks.
  ///
  /// @todo Merge this with edge_view specialization with is_edge_view mf.
  ////////////////////////////////////////////////////////////////////
  template<typename View>
  static
  std::size_t
  apply(std::pair<ptr_wrapper<View>, typename View::index_type> const&)
  {
    return 1;
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Counts the specificiation of aggregated consumed results of
  ///   preceeding tasks.
  ////////////////////////////////////////////////////////////////////
  template<typename T, typename... OptionalFilter>
  static
  std::size_t
  apply(std::pair<ptr_wrapper<aggregated_edge_view<T, OptionalFilter...>>,
                       std::vector<std::size_t> > const& vid)
  {
    return vid.second.size();
  }
}; // struct count_df_preds_func


//////////////////////////////////////////////////////////////////////
/// @brief Creates the notifiers necessary to flow the result of a
///   non-explicitly dependent task (i.e., those specified via consume)
///   so it is available for the task being added on this location.
/// @ingroup paragraph
///
/// @see task_graph::add_task
//////////////////////////////////////////////////////////////////////
template<typename NotifierPtr>
struct setup_edge_flow_func
{
  NotifierPtr               m_notifier_ptr;
  executor_base*            m_exec_ptr;
  edge_container&           m_edge_ct;

  setup_edge_flow_func(NotifierPtr const notifier_ptr,
                       executor_base* const exec_ptr,
                       edge_container& edge_ct)
    : m_notifier_ptr(notifier_ptr),
      m_exec_ptr(exec_ptr),
      m_edge_ct(edge_ct)
  { }

  template<typename Spec>
  void operator()(Spec const&) const
  { }

  ////////////////////////////////////////////////////////////////////
  /// @brief Create flow for a consumed result from a task within the PARAGRAPH.
  ////////////////////////////////////////////////////////////////////
  template<typename View>
  void operator()(std::pair<ptr_wrapper<View>,
                  typename View::index_type> const& vid) const
  {
    vid.first->setup_flow(vid.second, m_notifier_ptr, m_edge_ct);
  }
}; // struct setup_edge_flow_func

} // namespace paragraph_impl


namespace detail {

//////////////////////////////////////////////////////////////////////
/// @class ephemeral_pg
/// @brief Conditional base class of task_graph_impl used for PARAGRAPHs that
///   destroy themselves incrementally as they execute.
/// @tparam Task class used to represent PARAGRAPH tasks.
/// @tparam SchedInfo Information required by the scheduler processing the
///   PARAGRAPH to order tasks.
/// @ingroup paragraph
///
/// @todo Determine if the Task parameter is needed or if task_base will always
///   be used.
//////////////////////////////////////////////////////////////////////
template<typename Task>
class ephemeral_pg
{
private:
#ifndef STAPL_NDEBUG
  /// Used to detect reinvocation of a non-persistant PARAGRAPH.
  bool                                 m_b_invoked;
#endif

  /// Stores the factory task and associated scheduling information.
  Task*                                m_factory_task_ptr;

public:
  ////////////////////////////////////////////////////////////////////
  /// @brief Stores the factory task with default scheduling information.
  ////////////////////////////////////////////////////////////////////
  ephemeral_pg(Task* const factory_task_ptr)
    :
#ifndef STAPL_NDEBUG
      m_b_invoked(false),
#endif
      m_factory_task_ptr(factory_task_ptr)
  { }

  ephemeral_pg(ephemeral_pg const& other)
    :
#ifndef STAPL_NDEBUG
      m_b_invoked(false),
#endif
      m_factory_task_ptr(other.m_factory_task_ptr)
  {
#ifndef STAPL_NDEBUG
    stapl_assert(!other.m_b_invoked,
      "paragraph copy constr, pg previously invoked");
#endif
    stapl_assert(m_factory_task_ptr != nullptr,
      "paragraph copy constr, factory_task_ptr not set");
  }

  ~ephemeral_pg()
  {
    stapl_assert(m_factory_task_ptr != nullptr,
      "~ephemeral_pg:null m_factory_task_ptr");
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Required to provide a consistent interface to the derived class.
  ///   Ephemeral PARAGRAPHs only have the factory task as their starting task.
  ////////////////////////////////////////////////////////////////////
  void add_starting_task(Task* const)
  { }

  ////////////////////////////////////////////////////////////////////
  /// @brief Return a range of a single task that contains the factory task.
  ////////////////////////////////////////////////////////////////////
  std::pair<Task**, Task**>
  get_executor_start_tasks(void)
  {
#ifndef STAPL_NDEBUG
    stapl_assert(!m_b_invoked, "~ephemeral_pgs aren't reinvokable");

    m_b_invoked = true;
#endif

    return std::make_pair(&m_factory_task_ptr, &m_factory_task_ptr + 1);
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Return the factory of the PARAGRAPH.
  ///
  /// @todo Determine if this is deadcode and kill it.
  ////////////////////////////////////////////////////////////////////
  Task& factory(void) const
  {
    stapl_assert(m_factory_task_ptr != nullptr, "null m_factory_task_ptr");

    return *m_factory_task_ptr;
  }
}; // class ephemeral_pg


//////////////////////////////////////////////////////////////////////
/// @class persistent_pg
/// @brief Conditional base class of task_graph_impl that maintains all tasks
///   and edges persistent in memory after one execution so that it can be
///   reused.
/// @ingroup paragraph
//////////////////////////////////////////////////////////////////////
template<typename Task>
class persistent_pg
{
private:
  /// Prevents copy constructing a persistant PARAGRAPH after it's been invoked.
  bool                                               m_b_invoked;

  /// Factory task and associated scheduling information.
  Task*                                              m_factory_task_ptr;

  /// Initial set of tasks and scheduling information spawned by the factory.
  std::vector<Task*>                                 m_starting_tasks;

public:
  ////////////////////////////////////////////////////////////////////
  /// @brief Stores the factory task with default scheduling information.
  ////////////////////////////////////////////////////////////////////
  persistent_pg(Task* const factory_task_ptr)
    : m_b_invoked(false), m_factory_task_ptr(factory_task_ptr)
  { }

  persistent_pg(persistent_pg const& other)
    : m_b_invoked(false), m_factory_task_ptr(other.m_factory_task_ptr)
  {
    stapl_assert(!other.m_b_invoked,
      "paragraph copy constr, pg previously invoked");

    stapl_assert(m_starting_tasks.size() == 0,
      "paragraph copy constr, m_starting_tasks not empty");

    stapl_assert(m_factory_task_ptr != nullptr,
      "paragraph copy constr, factory_task_ptr not set");
  }

  ////////////////////////////////////////////////////////////////////
  /// @todo if we can downcast task_base* to factory_task* then we
  ///   can prevent ~task_base() from being virtual (or make a
  ///   polymorphic deleter for this case...).
  ////////////////////////////////////////////////////////////////////
  ~persistent_pg()
  {
    stapl_assert(m_factory_task_ptr != nullptr,
      "~persistent_pg:null m_factory_task_ptr");

    for (Task* ptr : m_starting_tasks)
      delete ptr;
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief add a task and its scheduling informtation to the set of initial
  ///   tasks.
  ////////////////////////////////////////////////////////////////////
  void add_starting_task(Task* const t)
  {
    m_starting_tasks.push_back(t);
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Return the set of initial tasks to populate the PARAGRAPH
  ///   before it is reinvoked.
  ////////////////////////////////////////////////////////////////////
  std::pair<Task**, Task**>
  get_executor_start_tasks(void)
  {
    if (!m_b_invoked)
    {
      m_b_invoked = true;

      return std::make_pair(&m_factory_task_ptr, &m_factory_task_ptr + 1);
    }

    return std::make_pair(&(*m_starting_tasks.begin()),
                          &(*m_starting_tasks.end()));
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Return the factory of the PARAGRAPH.
  ///
  /// @todo Determine if this is deadcode and kill it.
  ////////////////////////////////////////////////////////////////////
  Task& factory(void) const
  {
    stapl_assert(m_factory_task_ptr != nullptr, "null m_factory_task_ptr");

    return *m_factory_task_ptr;
  }
}; // class persistent_pg


//////////////////////////////////////////////////////////////////////
/// @class task_graph_base
/// @brief Intermediate class in PARAGRAPH class hierarchy responsible for
///   handling registration of the instantiaion with the runtime.
/// @ingroup paragraph
///
/// In addition to object registration this class is responsible for removing
/// get_location_id() from the public interface of the task_graph.
//////////////////////////////////////////////////////////////////////
class task_graph_base
  : public p_object
{
protected:
  ////////////////////////////////////////////////////////////////////
  /// @brief Controls registration of the PARAGRAPH instance with the runtime.
  ///
  /// Setup p_object such that communication from this object does not buffer
  /// and is not included in bookkeeping for fences.
  ////////////////////////////////////////////////////////////////////
  task_graph_base()
    : p_object(no_aggregation | no_fence_information)
  { }

  virtual ~task_graph_base() override = default;
};

} // namespace detail


namespace paragraph_impl {

struct trivial_p_object
  : public p_object
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Contains all functionality of the PARAGRAPH that isn't dependent on
///   the Scheduler or View types.
/// @ingroup paragraph
//////////////////////////////////////////////////////////////////////
class task_graph
  : public detail::task_graph_base
{
  /// @todo Determine if this is still needed and remove it.
  friend class stapl::edge_container;

public:
  using task_id_t = std::size_t;

private:
  // Number of tasks processed on this location.
  std::size_t                                  m_processed_count;

  // Number of tasks added on this location.
  std::size_t                                  m_added_count;

  // Number of tasks that were placed on this location for execution
  // Used to delay termination detection attempts until all local tasks
  // have been executed (i.e., m_local_added_count == m_processed_count).
  std::size_t                                  m_local_added_count;

protected:
  // Tracks whether any task added (or passed here from another location)
  // is moved elsewhere for execution.
  bool                                         m_b_no_tasks_escaped;

  /// The executor for this PARAGRAPH.
  executor_base*                               m_exec_ptr;

  /// Indicate if the PARAGRAPH is blocking.
  bool                                         m_b_blocking;

  /// The number of invocations of this paragraph that have been started.
  unsigned int                                 m_execs_started;

  /// The number of invocations of this paragraph that have been finished.
  unsigned int                                 m_execs_terminated;

private:
  /// Indicates if the PARAGRAPH execution has terminated or not.
  bool                                         m_b_terminated;

  /// Flag to keep track if migration is enabled.
  bool                                         m_b_migration;

  /// Indicates if the reset method has been invoked.
  bool                                         m_b_reset_called;

  /// The successor count of this paragraph in its parent paragraph.
  std::size_t                                  m_num_succs;

  /// Boolean stating whether this paragraph has either predecessor or
  /// successor paragraph with dataflow connections.  Used to guard
  /// registration / unregistration with the parent paragraph.
  bool                                         m_b_participates_in_dataflow;

  rmi_handle::reference                        m_parent_handle_ref;

  location_type                                m_parent_location;

  bool                                         m_b_parent_notify_all_locations;

  size_t                                       m_task_id;

  /// Holds no successor task p_object with gang.  This variable is created
  /// on demand, in place. Attempts to relax memory consistency for such
  /// tasks, deferring the blocking for traffic completion until PARAGRAPH
  /// termination detection.
  boost::optional<trivial_p_object>            m_no_succs_task_gang;

  /// Holds successor task p_object with gang.  This variable is created
  /// on demand, in place. Reuse this gang for tasks with successors, where
  /// memory consistency is immediately enforced with a fence.
  boost::optional<trivial_p_object>            m_succs_task_gang;

  nested_pg_directory                          m_nested_directory;


  //////////////////////////////////////////////////////////////////////
  /// @brief Dispatched to @ref result_container to check if the PARAGRAPH's
  /// output ports have any consumers that have yet to request flow.
  /// Used as part of the condition for termination detection.
  //////////////////////////////////////////////////////////////////////
  virtual bool has_outstanding_consumers(void) const = 0;


public:
  bool participates_in_dataflow(void) const
  { return m_b_participates_in_dataflow; }


  //////////////////////////////////////////////////////////////////////
  /// @brief Forward dataflow setup request from child PARAGRAPH to the
  /// nested PARAGRAPH directory data member.
  ///
  /// @tparam T The type of elements stored in the target producer's
  ///   output port.
  ///
  /// @param producer Task identifier of the producer PARAGRAPH in its
  ///   enclosing / parent PARAGRAPH.
  ///
  /// @param consumer Task identifier of the consumer PARAGRAPH requesting
  ///   dataflow.
  ///
  /// @param consumer_handle RMI handle to the consuming PARAGRAPH.
  ///
  /// @param consumer_location location in the consuming PARAGRAPH initiating
  ///   the point to point consumption request.
  ///
  /// @param port_filter Used to select which pins of the producer port
  ///   are forwarded to this consumer.
  ///
  /// @param value_filter Applied to each flowed pin's value, prior to
  ///   dataflow to the consumer.
  ///
  /// @param location_mapper maps producer pin to location in this consumer
  ///   where it should be flowed.
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename PortFilter,
           typename ValueFilter, typename LocationMapper>
  void child_setup_flow(size_t producer, size_t consumer,
                        rmi_handle::reference consumer_handle,
                        location_type consumer_location,
                        PortFilter port_filter,
                        ValueFilter value_filter,
                        LocationMapper location_mapper)
  {
    this->nested_pgs().setup_flow<T>(
      this->get_location_id(),
      producer, consumer, consumer_handle, consumer_location,
      std::move(port_filter),
      std::move(value_filter),
      std::move(location_mapper));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Called by child paragraph locations to initiate registration
  /// in parent's directory of nested paragraphs.
  //////////////////////////////////////////////////////////////////////
  void add_nested_pg(tuple<std::size_t, location_type> key,
                     rmi_handle::reference producer_handle)
  {
    gang g(*this);
    this->nested_pgs().add_pg(key, producer_handle);
  }


  /// Edge container storing the results of tasks.
  /// @todo friend task_factory_base and make this a protected member.
  edge_container*                              m_edge_ct_ptr;

  //////////////////////////////////////////////////////////////////////
  /// @brief Get a reference to the output port of this PARAGRAPH.
  /// Primarily used to forward requests from consumers in a type
  /// erased manner.
  //////////////////////////////////////////////////////////////////////
  virtual detail::result_container_base& get_result_container(void) = 0;


  //////////////////////////////////////////////////////////////////////
  /// @brief Return true if PARAGRAPH scheduler has signalled that all
  /// tasks are locally executed.
  //////////////////////////////////////////////////////////////////////
  virtual bool has_only_local_tasks(void) const                      = 0;

  virtual bool has_span(void) const                                  = 0;

  //////////////////////////////////////////////////////////////////////
  /// @brief Forward sibling PARAGRAPH dataflow request to
  /// @ref result_container (the output port of this PARAGRAPH).
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename PortFilter, typename ValueFilter,
           typename LocationMapper>
  void request_out_flow(std::size_t consumer,
                        rmi_handle::reference consumer_handle,
                        location_type parent_location,
                        PortFilter port_filter,
                        ValueFilter value_filter,
                        LocationMapper location_mapper)
  {
    auto& result_ct =
      down_cast<detail::result_container<T>&>(this->get_result_container());

    result_ct.request_flow(consumer, consumer_handle, parent_location,
                           std::move(port_filter), std::move(value_filter),
                           std::move(location_mapper));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Forward parent PARAGRAPH output port dataflow request to
  /// @ref result_container (the output port of this PARAGRAPH).
  //////////////////////////////////////////////////////////////////////
  template<typename Mapper>
  void request_parent_out_flow(Mapper mapper)
  {
    this->get_result_container().request_parent_flow(
      mapper, [mapper](size_t index) { return mapper.should_flow(index); }
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Receive dataflow for child paragraph location on this affinity
  /// and forward to it.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void
  child_nested_sibling_set_element_recv(rmi_handle::reference child_consumer,
                                        location_type child_location,
                                        size_t index, T const& val)
  {
    auto child_ptr = resolve_handle<edge_container>(child_consumer);

    stapl_assert(child_ptr != nullptr, "Failed to resolve child handle");

    stapl_assert(child_ptr->get_location_id() == child_location,
                 "set element arrivated at unexpected child location");

    child_ptr->nested_sibling_set_element<T>(index, val);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Forward dataflow from a given index of a producer PARAGRAPH
  /// output port to the input port on this PARAGRAPH with the specified
  /// handle.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void
  child_nested_sibling_set_element_send(location_type parent_location,
                                        rmi_handle::reference child_consumer,
                                        location_type child_location,
                                        size_t index, T const& val)
  {
    gang g(*this);

    async_rmi(parent_location, this->get_rmi_handle(),
              &task_graph::child_nested_sibling_set_element_recv<T>,
              child_consumer, child_location, index, val);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Functor used to facilitate dataflow from producer to consumer
  /// paragraph when target consumer location to parent location mapping
  /// is not known on the producer side.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  class child_data_flow_func
  {
  private:
    rmi_handle::reference m_handle;
    size_t                m_index;
    T                     m_val;

  public:
    child_data_flow_func(rmi_handle::reference handle,
                         size_t index, T const& val)
      : m_handle(handle), m_index(index), m_val(val)
    { }

    template<typename Key>
    void operator()(p_object& d, Key const& key) const
    {
      auto& pg_directory = down_cast<nested_pg_directory&>(d);

      pg_directory.m_tg_ptr->child_nested_sibling_set_element_recv(
        m_handle, get<1>(key), m_index, m_val);
    }

    void define_type(typer& t)
    {
      t.member(m_handle);
      t.member(m_index);
      t.member(m_val);
    }
  };


  //////////////////////////////////////////////////////////////////////
  /// @brief Routes dataflow to the specified child paragraph at the
  /// location (in child gang) specified.  This function is called on
  /// the parent paragraph.
  ///
  /// @param child Task id of the child in parent paragraph.
  /// @param child_handle rmi handle of the input port of the child.
  ///   paragraph where data is headed.
  /// @param child_location Location in child's gang where dataflow
  ///   is headed.
  /// @param index The pin index to flow data through in child input port.
  /// @param val The value being flowed.
  ///
  /// @todo The standard @p invoke_where method of the directory incurs
  ///   an indirection / forward of this message through the managing
  ///   location for the given key (in this case child / child_location).
  ///   For large value payloads, it would be better to fetch the destination
  ///   location and then direct send.  This violates arrival ordering of
  ///   messages, but is not of concern for dataflow messages.  The directory
  ///   should be extended with an appropriate message / flag and then this
  ///   function updated accordingly.
  ///
  /// @todo gang switching directly with the @p this fails to appropriately
  ///   change the ARMI context.  Using resolve handle seems workaround this
  ///   problem but investigation into the ARMI cause of this is needed.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void child_route_dataflow(std::size_t child,
                            rmi_handle::reference child_handle,
                            location_type child_location,
                            size_t index, T const& val)
  {
    // see todo above.  There appears to be a runtime failure to gang
    // switch with the typical gang g(*this) idiom.
    gang g(*resolve_handle<p_object>(this->get_rmi_handle()));

    this->nested_pgs().invoke_where(
      child_data_flow_func<T>(child_handle, index, val),
      make_tuple(child, child_location));
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Returns true if the given PARAGRAPH has been called at least
  /// once before (i.e., this is a reinvocation).
  //////////////////////////////////////////////////////////////////////
  bool called_before(void) const
  {
    return this->m_execs_terminated != 0;
  }

protected:
  template<typename TaskMapperParam>
  task_graph(TaskMapperParam&& task_mapper_param,
             bool b_enable_migration,
             bool b_persistent,
             std::size_t num_succs,
             bool b_participates_in_dataflow,
             rmi_handle::reference parent_handle_ref,
             location_type parent_location,
             bool b_parent_notify_all_locations,
             size_t task_id)
    : m_processed_count(0),
      m_added_count(0),
      m_local_added_count(0),
      m_b_no_tasks_escaped(true),
      m_exec_ptr(nullptr),
      m_b_blocking(true),
      m_execs_started(0),
      m_execs_terminated(0),
      m_b_terminated(false),
      m_b_migration(b_enable_migration),
      m_b_reset_called(false),
      m_num_succs(num_succs),
      m_b_participates_in_dataflow(b_participates_in_dataflow),
      m_parent_handle_ref(std::move(parent_handle_ref)),
      m_parent_location(parent_location),
      m_b_parent_notify_all_locations(b_parent_notify_all_locations),
      m_task_id(task_id),
      m_nested_directory(this, task_mapper_param),
      m_edge_ct_ptr(
        new edge_container(b_enable_migration, this, b_persistent,
                           std::forward<TaskMapperParam>(task_mapper_param)))
  {
    // Initialize the two task communication groups.
    {
      gang g;
      m_succs_task_gang = boost::in_place<trivial_p_object>();
    }

    {
      gang g;
      m_no_succs_task_gang = boost::in_place<trivial_p_object>();
    }
  }

  task_graph(task_graph const&)            = delete;
  task_graph& operator=(task_graph const&) = delete;

  ////////////////////////////////////////////////////////////////////
  /// @brief Return whether the PARAGRAPH has terminated or not.
  ////////////////////////////////////////////////////////////////////
  bool finished(void) const
  {
    return m_b_terminated;
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief The portion of the paragraph function operator not dependent
  /// on any template parameters.  Called by @ref paragraph::operator().
  ///
  /// @todo block_until for TD notification can be removed when per pg
  /// invocation metadata is created (needed for more general support of
  /// overlapping pg invocations.
  ////////////////////////////////////////////////////////////////////
  void
  operator()(int unused, bool blocking, bool force_evaluation, bool one_sided)
  {
    // Make sure TD notification from previous iteration arrives prior to
    // starting this iteration so that metadata reset in reset() is not
    // corrupted by late arriving TD notification.
    block_until([this]()
      { return this->m_execs_started == this->m_execs_terminated; });

    const bool b_called_before = this->called_before();

    stapl_assert(!b_called_before || this->is_persistent(),
      "paragraph::operator(): recall on ephemeral pg");

    stapl_assert(!b_called_before || (this->is_blocking() == blocking),
      "paragraph::operator(): recall change blocking bit");

    stapl_assert(!b_called_before || this->finished(),
      "paragraph::operator():: recall before finished");

    stapl_assert(one_sided == this->parent_handle().valid(),
      "Unexpected nested / m_parent_handle_ref state mismatch");

    if (!b_called_before)
    {
      // For the Factory task on the first time through. Subsequent calls for
      // persistent paragraph inserts tasks that are already counted
      this->count_task();
      this->count_local_task();

      // set up data member to denote if the task graph is blocking.
      this->m_b_blocking = blocking;
    }
    else
    {
      this->advance_epoch();
      this->reset();
    }

    ++this->m_execs_started;
  }


public:
  ////////////////////////////////////////////////////////////////////
  /// @todo the edge_container may need to stay alive longer even if the local
  ///   work is done (i.e., other locations with consumers haven't notified yet,
  ///   which is beyond this location's control).
  ////////////////////////////////////////////////////////////////////
  ~task_graph() override
  {
    if (this->participates_in_dataflow())
    {
      auto* parent_pg_ptr = resolve_handle<task_graph>(parent_handle());

      stapl_assert(parent_pg_ptr != nullptr, "Failed Resolution");

      gang g(*parent_pg_ptr);

      parent_pg_ptr->nested_pgs().unregister_key(
        make_tuple(this->task_id(), this->get_location_id()));
    }

    stapl_assert(m_execs_started == m_execs_terminated,
      "paragraph deletion attempted with num_execs / num_terminated mismatch");
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Return successor task p_object used in gang switcher.
  ////////////////////////////////////////////////////////////////////
  trivial_p_object const& succs_p_object(void) const
  {
    return *m_succs_task_gang;
  }


  ////////////////////////////////////////////////////////////////////
  /// @brief Return no successor task p_object used in gang switcher.
  ////////////////////////////////////////////////////////////////////
  trivial_p_object const& no_succs_p_object(void) const
  {
    return *m_no_succs_task_gang;
  }

  bool has_successors(void) const
  {
    return m_num_succs > 0;
  }

  std::size_t num_successors(void) const
  {
    return m_num_succs;
  }

  rmi_handle::reference const& parent_handle(void) const
  {
    return m_parent_handle_ref;
  }

  location_type parent_location(void) const
  {
    return m_parent_location;
  }

  bool parent_notify_all_locations(void) const
  {
    return m_b_parent_notify_all_locations;
  }

  size_t task_id(void) const
  {
    return m_task_id;
  }

protected:
  virtual void reset_in_out_containers(void) = 0;

public:
  ////////////////////////////////////////////////////////////////////
  /// @brief Starts termination detection.
  ////////////////////////////////////////////////////////////////////
  virtual void pulse_terminator(void) = 0;

  edge_container& edges(void)
  { return *(this->m_edge_ct_ptr); }

  nested_pg_directory& nested_pgs(void)
  { return this->m_nested_directory; }

  nested_pg_directory const& nested_pgs(void) const
  { return this->m_nested_directory; }

  ////////////////////////////////////////////////////////////////////
  /// @brief Called by task wrapper of one-sided, persistent paragraphs
  /// to signal time for deletion.
  ///
  /// @todo Modify @ref notify_terminated to check for orphaned deletable
  /// status (switched on here) so that block_until() construct is not neded.
  ////////////////////////////////////////////////////////////////////
  void try_os_delete(void)
  {
    // Wait until TD notification arrives before deleting so that it doesn't
    // attempt to access freed memory.
    block_until([this]()
      { return this->m_execs_started == this->m_execs_terminated; });

    delete this;
  }


  ////////////////////////////////////////////////////////////////////
  /// @brief Indicate if the task_graph was instantiated with task migration
  ///   support enabled.
  ////////////////////////////////////////////////////////////////////
  bool migration_enabled() const
  {
    return m_b_migration;
  }


  ////////////////////////////////////////////////////////////////////
  /// @brief Indicate if the task_graph is persistent.
  ////////////////////////////////////////////////////////////////////
  virtual bool is_persistent(void) const = 0;

  ////////////////////////////////////////////////////////////////////
  /// @brief Return the factory task.
  ////////////////////////////////////////////////////////////////////
  virtual task_base& get_factory_task(void)             = 0;
  virtual task_base const& get_factory_task(void) const = 0;

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a pointer to the executor responsible for processing
  /// this instance.
  ///
  /// Use pointer to allow cases where inter-paragraph input ports receive
  /// dataflow prior to initialization of executor (they won't use it,
  /// as no intra-paragraph consumers could have been initialized as the
  /// factory hasn't been called yet).
  //////////////////////////////////////////////////////////////////////
  executor_base* executor_ptr(void) const
  {
    return m_exec_ptr;
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Return the executor responsible for processing this instance.
  ////////////////////////////////////////////////////////////////////
  executor_base& executor(void) const
  {
    stapl_assert(m_exec_ptr != nullptr, "tg::executor(): m_exec_ptr is null");

    return *m_exec_ptr;
  }


  ////////////////////////////////////////////////////////////////////
  /// @brief Receive the pointer to the executor from the PARAGRAPH function
  ///   operator.
  ////////////////////////////////////////////////////////////////////
  void set_executor(executor_base* exec_base_ptr)
  {
    m_exec_ptr = exec_base_ptr;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief For port flow PARAGRAPHs, connect the output of the given task
  /// to the given index of the output port.
  //////////////////////////////////////////////////////////////////////
  virtual void set_result(std::size_t, task_id_t) = 0;


  ////////////////////////////////////////////////////////////////////
  /// @brief Allows the factory to specify which task produces the PARAGRAPH
  ///   result on this location, which can be used to customize termination
  ///   detection.
  ////////////////////////////////////////////////////////////////////
  void set_result(task_id_t task_id)
  {
    this->set_result(this->get_location_id(), task_id);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Specify that a child PARAGRAPH with the given task id
  /// will connect to some of the ports of this parent PARAGRAPH as defined
  /// by the passed mapper.
  //////////////////////////////////////////////////////////////////////
  template<typename Mapper>
  void set_result_pg(task_id_t task_id, Mapper mapper)
  {
    this->nested_pgs().setup_parent_flow(task_id, mapper);
  }


  ////////////////////////////////////////////////////////////////////
  /// @brief Receive the request from a task to receive notification when a
  ///   task has been processed.
  /// @param tid Id of the task whose completion will trigger the notifier.
  /// @param notifier The notifier to invoke.
  ////////////////////////////////////////////////////////////////////
  template<typename Result, typename Notifier>
  void request_notify(size_t tid, Notifier const& notifier)
  {
    stapl_assert(!m_b_terminated, "tg::request_notifier found tg terminated");

    m_edge_ct_ptr->setup_signal(tid, notifier);
  }


  ////////////////////////////////////////////////////////////////////
  /// @brief Method invoked by a notifier when the PARAGRAPH has terminated.
  ////////////////////////////////////////////////////////////////////
  void notify_termination()
  {
    m_b_terminated = true;
    ++m_execs_terminated;

    if (this->is_persistent())
      return;

    m_edge_ct_ptr->release();

    if (!is_blocking())
      delete this;
  }


public:
  ////////////////////////////////////////////////////////////////////
  /// @brief Receive a task being migrated from another location.
  /// @param tid Id of the task
  /// @param unknown_consumer_cnt Number of successors that have not yet
  ///   requested notification of the task's completion or result.
  /// @param sched_info Information used to determine relative priority of task.
  /// @param wf work function the task will invoke.
  /// @param views The packed view of the migrated task.
  /// @todo Receiving parameter pack instead of tuple of views causes
  ///   compilation errors, probably due to view p_object packing.
  ///   Resolve and strip tuple usage.
  ////////////////////////////////////////////////////////////////////
  template<typename SchedulerEntry, typename WF,
           typename ViewSet, std::size_t ...Indices>
  void
  add_migrated_task(std::size_t tid, size_t unknown_consumer_cnt,
                    typename SchedulerEntry::sched_info_type const& sched_info,
                    WF const& wf, ViewSet const&,
                    index_sequence<Indices...> const&);


  ////////////////////////////////////////////////////////////////////
  /// @brief Increment the count of tasks placed on this location.
  ////////////////////////////////////////////////////////////////////
  void count_local_task(size_t cnt = 1)
  {
    m_local_added_count += cnt;
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Increment the count of tasks specified on this location.
  ///
  /// Called in paragraph_view::add_task() methods, which is entry point
  /// for both factories and dynamic_wfs
  ////////////////////////////////////////////////////////////////////
  void count_task(size_t cnt = 1)
  {
    m_added_count += cnt;
  }


  void processed_local(void)
  {
    ++m_processed_count;
    pulse_terminator();
  }


  ////////////////////////////////////////////////////////////////////
  /// @brief Receive notification from a task that it has executed.
  ///  Method is called by task_base::processed when the task has a void
  ///  return value and possibly intra-PARAGRAPH successors.
  ////////////////////////////////////////////////////////////////////
  void processed_remote_void(task_id_t tid, bool b_has_succs)
  {
    gang g(*this);

    if (b_has_succs)
      m_edge_ct_ptr->set_element(tid);

    ++m_processed_count;

    pulse_terminator();
  }


  ////////////////////////////////////////////////////////////////////
  /// @brief Receive notification from a task that it has executed.
  ///
  /// Called by Task objects with non-void return type.  The produced value is
  /// passed so that edge_container can forward it to consumers.
  /// @todo compute_df_edge_type is a temporary patch until generalized proxy
  ///   packing is implemented. It intercepts callbacks to here from
  ///   gang_executor (nested paragraph task migrations) that wrap proxies
  ///   in proxy holders for packing.
  ////////////////////////////////////////////////////////////////////
  template<typename ReturnValue>
  void processed_remote(task_id_t tid, bool has_succs, ReturnValue&& val)
  {
    if (has_succs)
    {
      typedef typename compute_df_edge_type<
        typename std::decay<ReturnValue>::type
      >::type edge_t;

      m_edge_ct_ptr->set_element<edge_t>(tid, std::forward<ReturnValue>(val));
    }

    ++m_processed_count;

    pulse_terminator();
  }


  ////////////////////////////////////////////////////////////////////
  /// @brief Invoked by the result notifier.
  ////////////////////////////////////////////////////////////////////
  void result_processed()
  {
    ++m_processed_count;
    pulse_terminator();
  }


  ////////////////////////////////////////////////////////////////////
  /// @brief Set the number of successors that will require notification
  ///   of a tasks execution.
  ///
  /// This method is used by dynamic skeletons where the number of successors of
  /// a task isn't known when the task is created (e.g., while loop skeleton).
  ////////////////////////////////////////////////////////////////////
  void set_num_succs(std::size_t task_id, std::size_t num_succs) const
  {
    m_edge_ct_ptr->set_num_succs(task_id, num_succs);
  }


  ////////////////////////////////////////////////////////////////////
  /// @brief Migrate a task to another location.
  /// @param task Task to be migrated
  /// @param dest Location to which task is to be migrated.
  /// @param sched_info Scheduling information of the task.
  ///
  /// Called by a task to migrate itself to another location as part of
  /// the process of work stealing.
  ///
  /// @todo Fix universal refs in @ref vs_map or trash it for variadic
  /// expansion so that move of tasks viewset into packaging can be done.
  ////////////////////////////////////////////////////////////////////
  template<typename SchedulerEntry, typename WF,
           typename ViewWrapper, std::size_t ...Indices>
  void migrate_task(Task<SchedulerEntry, std::true_type, std::false_type,
                      WF, ViewWrapper>* task,
                    std::size_t dest,
                    index_sequence<Indices...> const&)
  {
    m_b_no_tasks_escaped = false;

    --m_local_added_count;

    typedef Task<
      SchedulerEntry, boost::mpl::true_, boost::mpl::false_,
      WF, ViewWrapper>                                          task_t;
    typedef typename task_t::result_t                           result_t;

    // prepare the task for migration
    // apply the transformation
    typedef compute_demotion_wf<WF>                             selector_t;
    typedef typename selector_t::result_type                    wf_t;

    // Switch to PARAGRAPH communication group.
    gang g(*this);

    typedef decltype(
      vs_map(transporter_packager(), task->viewset().get_orig_vs())) packaged_t;

    typedef tuple<
              typename migration_packer<
                typename tuple_element<Indices, packaged_t>::type
              >::type...
            > tuple_t;

    typedef void (task_graph::* mem_fun_t)
      (std::size_t, size_t, typename SchedulerEntry::sched_info_type const&,
       wf_t const&, tuple_t const&, index_sequence<Indices...> const&);

    constexpr mem_fun_t mem_fun =
      &task_graph::add_migrated_task<SchedulerEntry, wf_t, tuple_t, Indices...>;

    stapl_assert(this->get_location_id() != dest,
                 "migrating task to source location");

    // task does not have succs, send directly to the dest and you are done.
    if (!task->has_succs())
    {
      async_rmi(dest, this->get_rmi_handle(), mem_fun,
                task->task_id(), 0, task->sched_info(),
                selector_t::apply(static_cast<WF const&>(*task)),
                tuple_t(migration_packer<
                  typename tuple_element<Indices, packaged_t>::type
                >::apply(transporter_packager()(get<Indices>(
                  task->viewset().get_orig_vs())), *m_edge_ct_ptr)...),
                index_sequence<Indices...>());

      vs_map(lazy_ref_release_func(tg_callback(this)), task->viewset().views());

      return;
    }

    // this is a producer task, need to migrate the entry as well.
    // forward the request to the container to migrate the task
    // along with proper handling of the entry.
    m_edge_ct_ptr->migrate_entry<result_t>(
      task->task_id(), dest,
      boost::bind(
        mem_fun, lazy_ref(*this),
        task->task_id(), _1, task->sched_info(),
        selector_t::apply(static_cast<WF const&>(*task)),
        tuple_t(migration_packer<
          typename tuple_element<Indices, packaged_t>::type
        >::apply(transporter_packager()(
          get<Indices>(task->viewset().get_orig_vs())), *m_edge_ct_ptr)...),
        index_sequence<Indices...>()));

    vs_map(lazy_ref_release_func(tg_callback(this)), task->viewset().views());
  }


  template<typename T>
  void
  setup_promise_flow(std::size_t task_id,
                     promise<typename df_stored_type<T>::type> pr)
  {
    gang g(*this);

    m_edge_ct_ptr->setup_promise_flow<T>(task_id, std::move(pr));
  }


  ////////////////////////////////////////////////////////////////////
  /// @brief Detects whether participation in global quiescence should
  /// be initiated, based on whether there are local conditions that
  /// would certainly cause it to fail (eg, all tasks in the local
  /// queue have not been retired).
  ///
  /// Used to guard initialization of TD in @ref pulse_terminator
  /// and as part of termination value in @ref termination_value.
  ////////////////////////////////////////////////////////////////////
  int hold_termination(void) const
  {
    if (m_processed_count != m_local_added_count)
      return 1;

    if (!this->get_factory_task().finished())
      return 1;

    // Checks that outbound dataflow from this location has
    // finished, both for intra and inter PARAGRAPH edges.
    //
    // not_intra_out_edges_fully_specified
    if (m_edge_ct_ptr->has_pending_consumers())
      return 1;

    // not_inter_out_edges_fully_specified
    if (this->has_outstanding_consumers())
      return 1;

    // If the no successor gang was created, query its fence metadata.
    // Otherwise, set to zero as it shouldn't affect termination detection.
    if (m_no_succs_task_gang)
    {
      auto const& location_md = this->no_succs_p_object().get_location_md();

      if (location_md.has_pending_rmis())
        return 1;
    }

    return 0;
  }


  ////////////////////////////////////////////////////////////////////
  /// @brief Return the termination value of the PARAGRAPH on this location.
  /// @return non-zero if there are outstanding tasks.
  ///
  /// Called by termination_detection.
  ////////////////////////////////////////////////////////////////////
  std::pair<int, bool> termination_value(void) const
  {
    return std::pair<int, bool>(
      m_added_count - m_processed_count + hold_termination(),
      m_b_no_tasks_escaped
    );
  }


  ////////////////////////////////////////////////////////////////////
  /// @brief Call back to the factory to allow for incremental construction of
  ///   the task graph.
  ///
  /// This method allows reinvocation of the factory without the task graph
  /// being aware of WF and View types.
  ///
  /// Invoked by executor when the scheduler on this location has no runnable
  /// tasks.  If the factory is finished no new tasks are added.
  ///
  /// @returns @c true if the factory needs to be queried again,
  ///          otherwise @c false.
  ////////////////////////////////////////////////////////////////////
  bool query_factory(void)
  {
#ifndef GFORGE_BUG_1443_RESOLVED
    bool keep_alive = this->get_factory_task().reinvoke(this);

    // Check to see if communication and metadata are complete.
    // If they're not, keep the PARAGRAPH active by reporting that the factory
    // needs to be reinvoked. This is a workaround to keep the PARAGRAPH
    // in the gang_executor scheduler so it is polled.  It can be removed
    // when the notification of communication completing is event-based and the
    // PARAGRAPH is able to register a callback with ARMI that is
    // invoked when the communication the PARAGRAPH depends on is complete.
    if (!keep_alive && m_no_succs_task_gang)
    {
      auto& location_md = this->no_succs_p_object().get_location_md();

      keep_alive = location_md.has_pending_rmis();
    }

   if (!keep_alive)
     this->pulse_terminator();

    return keep_alive;
#else
    return this->get_factory_task().reinvoke(this);
#endif
  }

protected:
  ////////////////////////////////////////////////////////////////////
  /// @brief Clear all counters and booleans modified by execution of the
  ///   PARAGRAPH to allow it to be reinvoked.
  ////////////////////////////////////////////////////////////////////
  void reset(void)
  {
    m_processed_count = 0;

    // The Factory isn't added in subsequent calls to persistent pg.  On first
    // call to reset set, decrement added task count to account for this in
    // subsquent bookkeeping for termination detect, etc.
    if (!m_b_reset_called)
    {
      --m_added_count;
      --m_local_added_count;
      m_b_no_tasks_escaped = true;
    }

    m_b_reset_called = true;
    m_b_terminated   = false;

    this->m_edge_ct_ptr->reset_entry_values();
    this->reset_in_out_containers();
  }

public:
  ////////////////////////////////////////////////////////////////////
  /// @todo Remove this method and edge_container::empty()
  ////////////////////////////////////////////////////////////////////
  bool local_empty(void) const
  {
    return m_edge_ct_ptr->empty();
  }

  bool is_blocking(void) const
  {
    return m_b_blocking;
  }

  typedef void stapl_is_paragraph_;

}; // class task_graph


//////////////////////////////////////////////////////////////////////
/// @class task_graph_impl
/// @brief Contains functionality of the PARAGRAPH that is dependent on the
///   the type of scheduler used in the executor processing the PARAGRAPH,
///   though not the Views of the PARAGRAPH.
/// @ingroup paragraph
///
/// PARAGRAPH persistence is enabled by a typedef reflected by the scheduler.
/// task_graph_impl derives from persistent_pg or ephemeral_pg depending on
/// whether the typedef is reflected by the scheduler or not.
//////////////////////////////////////////////////////////////////////
template<typename Scheduler>
class task_graph_impl
  : public task_graph,
    public std::conditional<
      has_enable_persistence<Scheduler>::value,
      detail::persistent_pg<
        task_base_intermediate<typename Scheduler::entry_type>>,
      detail::ephemeral_pg<
        task_base_intermediate<typename Scheduler::entry_type>>
    >::type
{
public:
  using sched_info_type  = typename Scheduler::sched_info_type;
  using sched_entry_type = typename Scheduler::entry_type;
  using scheduler_type   = Scheduler;
  using task_type        = task_base_intermediate<sched_entry_type>;

private:
  /// Container used to specify the set of predecessor task ids of a task.
  using pred_list_t      = std::vector<std::size_t>;

  /// Determine the persistence-oriented base class from which this instance
  /// derives.
  typedef typename std::conditional<
    has_enable_persistence<Scheduler>::value,
    detail::persistent_pg<task_type>,
    detail::ephemeral_pg<task_type>
  >::type                                           persister_t;

  scheduler_type m_scheduler;

protected:
  persister_t& persister(void)
  {
    return static_cast<persister_t&>(*this);
  }

  persister_t const& persister(void) const
  {
    return static_cast<persister_t const&>(*this);
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Construct the task graph with task migration information.
  /// @param task_mapper_param Mapper used to map task ids to location
  ///   managing them in the directory (base class of @ref edge_container).
  /// @param factory_task_ptr Pointer to the factory task for the PARAGRAPH.
  /// @param scheduler The scheduler instance passed by the user.
  /// @param b_enable_migration Specifies whether this instance supports task
  ///   migration or not.
  /// @param num_succs Number of successors this PARAGRAPH has.
  /// @param b_participates_in_dataflow Denotes whether this paragraph has
  ///   either predecessor or successor paragraph with dataflow connections.
  /// @param parent_handle_ref rmi_handle reference to the parent paragraph.
  /// @param parent location The location in the parent paragraph's gang
  ///   managing this nested paragraph task.
  /// @param task_id The task identifier assigned to this PARAGRAPH by its
  ///   parent.
  ////////////////////////////////////////////////////////////////////
  template<typename TaskMapperParam>
  task_graph_impl(TaskMapperParam&& task_mapper_param,
                  task_type* factory_task_ptr,
                  scheduler_type scheduler,
                  bool b_enable_migration,
                  std::size_t num_succs,
                  bool b_participates_in_dataflow,
                  rmi_handle::reference parent_handle_ref,
                  location_type parent_location,
                  bool b_parent_notify_all_locations,
                  size_t task_id)
    : task_graph(std::forward<TaskMapperParam>(task_mapper_param),
                 b_enable_migration,
                 has_enable_persistence<Scheduler>::value,
                 num_succs,
                 b_participates_in_dataflow,
                 std::move(parent_handle_ref),
                 parent_location,
                 b_parent_notify_all_locations,
                 task_id),
      persister_t(factory_task_ptr),
      m_scheduler(std::move(scheduler))
  {
    initialize_migrating_scheduler<Scheduler>::apply(m_scheduler, *this);
  }

  ~task_graph_impl() override
  {
    // NOTE - putting it here so can test is_persistent with virtual call...
    if (this->m_edge_ct_ptr != nullptr && this->is_persistent())
    {
      m_edge_ct_ptr->release();
    }
  }

  Scheduler& passed_scheduler(void)
  {
    return m_scheduler;
  }

  Scheduler& current_scheduler(void)
  {
    return down_cast<stapl::executor<task_graph_impl>*>
             (&(this->executor()))->scheduler();
  }

private:
  task_graph_impl& operator=(task_graph_impl const& other);

public:
  bool has_only_local_tasks(void) const override
  {
    return has_task_placement_all_local<Scheduler>::value;
  }

  using persistent_t =
    std::integral_constant<bool, has_enable_persistence<Scheduler>::value>;

  using migration_t = typename has_enable_migration<sched_info_type>::type;

  bool is_persistent(void) const final
  {
    return has_enable_persistence<Scheduler>::value;
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Construct non-factory tasks of the PARAGRAPH.
  /// @param tid Task id.
  /// @param wf The new task's workfunction.
  /// @param si Information to determine relative priority of task.
  /// @param pred_list List of tasks from which the task requires signals,
  ///   but not results.
  /// @param num_succs Number of tasks that will depend on this task.
  /// @param qualifier Indicates whether the task placement algorithm needs to
  ///   be run on this location.
  /// @param args Specification of the view elements passed to the task
  ///   invocation.
  ////////////////////////////////////////////////////////////////////
  template<typename WF, typename ExplicitPreds, typename ...Args>
  typename std::enable_if<!is_factory<WF>::value, void>::type
  add_task(std::size_t tid, WF const& wf, sched_info_type const& si,
           ExplicitPreds const& pred_list,
           std::size_t num_succs,
           loc_qual qualifier, Args const&... args);


  ////////////////////////////////////////////////////////////////////
  /// @brief Construct a nested PARAGRAPH task.
  /// @param tid Task id.
  /// @param f The new PARAGRAPH's factory workfunction.
  /// @param si Information to determine relative priority of task.
  /// @param pred_list List of tasks from which the task requires signals,
  ///   but not results.
  /// @param num_succs Number of tasks that will depend on this task.
  /// @param qualifier Indicates whether the task placement algorithm needs to
  ///   be run on this location. Not used for this signature.
  /// @param args Specification of the view elements passed to the task
  ///   invocation.
  /// @todo Refactor to further reduce duplicated code between the two
  ///   @p add_task signatures.
  ////////////////////////////////////////////////////////////////////
  template<typename Factory, typename ExplicitPreds, typename ...Args>
  typename std::enable_if<is_factory<Factory>::value, void>::type
  add_task(std::size_t tid, Factory const& f, sched_info_type const& si,
           ExplicitPreds const& pred_list, std::size_t num_succs,
           loc_qual qualifier, Args const&... args);
}; // class task_graph_impl

} // namespace paragraph_impl

} // namespace stapl

#include <stapl/views/metadata/partitioned_mix_view.hpp>

namespace stapl {

namespace paragraph_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Notifier used for tasks created using @ref transporter,
/// allowing them to wait on predecessor data flow.
//////////////////////////////////////////////////////////////////////
class transporter_notifier
{
private:
  tg_callback      m_cb;
  task_base*       m_task_ptr;
  std::size_t      m_pred_count;

  /// @brief This @ref p_object keeps the gang alive between task creation
  /// and task execution, avoiding the need for a fence between these two
  /// phases.
  trivial_p_object m_gang_sustainer;

  //////////////////////////////////////////////////////////////////////
  /// @brief Execute the held task when dependences are satisfied.
  ///
  /// @todo Using a promise based tg_callback solution would allow removal
  /// of rmi_fence().  Currently, closing the local communication group
  /// requires all traffic to have terminated.  However, if we implement the
  /// optimization for the case when a local paragraph representative is found,
  /// then we must consider how to break/transfer usage of the created promise.
  /// Alternatively, ARMI could allow a tag on rmis denoting no fence
  /// bookkeeping is necessary.
  //////////////////////////////////////////////////////////////////////
  void invoke(void)
  {
    gang g(m_gang_sustainer);

    m_task_ptr->operator()(m_cb);
    rmi_fence();
    delete this;
  }

public:
  transporter_notifier(tg_callback cb)
    : m_cb(std::move(cb)), m_task_ptr(nullptr), m_pred_count(0)
  { }

  STAPL_USE_MANAGED_ALLOC(transporter_notifier)

  //////////////////////////////////////////////////////////////////////
  /// @brief Called during task setup phase to have notifier wait for
  /// an additional predecessor data flow.
  //////////////////////////////////////////////////////////////////////
  void add_predecessor(void)
  {
    ++m_pred_count;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Called each time a promise is fulfilled for predecessor
  /// data flow.  If all flow received, execute the task.
  //////////////////////////////////////////////////////////////////////
  void notify_predecessor_available(void)
  {
    stapl_assert(m_pred_count, "m_pred_count is already 0");

    if ((--m_pred_count == 0) && (m_task_ptr != nullptr))
      this->invoke();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Called at end of task initialization.  If all data flow
  /// has been received during initialization, execute the task.
  /// @return True if task was immediately invoked, false if it deferred.
  //////////////////////////////////////////////////////////////////////
  void set_task_ptr(task_base* task_ptr)
  {
    m_task_ptr = task_ptr;

    if (m_pred_count > 0)
      return;

    this->invoke();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Implementation class of @ref edge_unpackager, allowing partial
/// specialization  to intercept proxies backed by an @ref edge_accessor.
/// Primary template is for non matches and is an identity operation.
//////////////////////////////////////////////////////////////////////
template<typename View>
struct edge_unpackager_impl
{
  typedef View type;

  static View& apply(View& view, rmi_handle::reference const&,
                     location_type, transporter_notifier*, p_object*)
  {
    return view;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Partial specialization matching @ref edge_accessor proxies.
/// Get predecessor task id from packaged proxy and initialize a new
/// proxy with storage that will be initialized with a promise
/// fulfilled by predecessor data flow.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct edge_unpackager_impl<lazy_edge_reference<T>>
{
  typedef lazy_edge_reference<T>                       type;

  static type apply(type& p,
                    rmi_handle::reference const& handle_ref,
                    location_type location,
                    transporter_notifier* notifier_ptr,
                    p_object* locality_ptr)
  {
    gang g(*locality_ptr);

    typedef T                                         value_type;
    typedef typename df_stored_type<value_type>::type stored_value_type;

    const size_t task_id = p.index();

    // (1) Create promise, send request to paragraph
    //     to fill with predecessor edge flow.
    promise<stored_value_type> pr;
    auto f = pr.get_future();

    typedef void (task_graph::* mem_fun_t)
      (std::size_t, promise<stored_value_type>);

    constexpr mem_fun_t mem_fun = &task_graph::setup_promise_flow<value_type>;

    async_rmi(location, handle_ref, mem_fun, task_id, std::move(pr));

    // (2) Create a new edge_accessor with a pointer to storage
    //     allocated for the data flow.
    auto entry_ptr = new detail::basic_edge_entry<value_type>(task_id);

    // (3) Register callback with future to promise that will:
    //   (a) Extract value and initialize storage backing edge_accessor.
    //   (b) call local notifier to tick down predecessor count
    notifier_ptr->add_predecessor();

    f.async_then(
      [entry_ptr, notifier_ptr, handle_ref](future<stored_value_type> f)
      {
        entry_ptr->set_value(f.get());
        notifier_ptr->notify_predecessor_available();
      });

    return type(typename type::constructor_param_type(entry_ptr, entry_ptr));
  }
};


template<typename T>
struct edge_unpackager_impl<pg_lazy_edge_reference<T>>
{
  using type = pg_lazy_edge_reference<T>;

  static type apply(type&, rmi_handle::reference const&, location_type,
                    transporter_notifier*, p_object*)
  {
    abort("transporting pg_lazy_edge_reference not supported");
    return type(0);
  }
};


class edge_unpackager
{
private:
  rmi_handle::reference   m_handle_ref;
  location_type           m_location;
  transporter_notifier*   m_notifier_ptr;

  /// @brief The @ref p_object which drove this out of paragraph gang task
  ///  placement.  Any edge_container related rmi traffic will be placed
  ///  in this object's gang, via the gang switching mechanism.
  p_object*               m_locality_ptr;

public:
  edge_unpackager(rmi_handle::reference handle_ref,
                  location_type location,
                  transporter_notifier* notifier_ptr,
                  p_object* locality_ptr)
    : m_handle_ref(std::move(handle_ref)),
      m_location(location),
      m_notifier_ptr(notifier_ptr),
      m_locality_ptr(locality_ptr)
  { }

  edge_unpackager(edge_unpackager const&) = delete;

  template<typename Signature>
  struct result;

  template<typename VS>
  struct result<edge_unpackager(VS)>
    : edge_unpackager_impl<
        typename std::remove_reference<VS>::type
      >
  { };

  template<typename VS>
  typename result<edge_unpackager(VS)>::type
  operator()(VS&& vs) const
  {
    return edge_unpackager_impl<
      typename std::remove_reference<VS>::type
    >::apply(vs, m_handle_ref, m_location, m_notifier_ptr, m_locality_ptr);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor pass to @ref executor_rmi for out of group task
///   placement / execution.
/// @tparam WF The task's workfunction.
/// @tparam VS The task's packed viewset.
/// @tparam SchedInfo The scheduling metadata for the task.
/// @todo Avoid heap allocation of notifier when there are no predecessor
///   data flows to wait on.
//////////////////////////////////////////////////////////////////////
template<typename WF, typename VS, typename SchedulerEntry,
         typename = make_index_sequence<tuple_size<VS>::value>>
class transporter;


template<typename WF, typename VS, typename SchedulerEntry,
         std::size_t ...Indices>
class transporter<WF, VS, SchedulerEntry, index_sequence<Indices...>>
{
private:
  typedef typename SchedulerEntry::sched_info_type sched_info_type;

  rmi_handle::reference  m_handle_ref;
  location_type          m_location;
  std::size_t            m_tid;
  WF                     m_wf;
  VS                     m_vs;
  sched_info_type        m_si;
  bool                   m_b_has_succs;
  rmi_handle::reference  m_locality_handle_ref;

public:
   transporter(rmi_handle::reference handle, location_type location,
               std::size_t tid, WF wf, VS spec, sched_info_type si,
               bool b_has_succs, rmi_handle::reference locality_handle)
    : m_handle_ref(std::move(handle)), m_location(location),
      m_tid(tid), m_wf(std::move(wf)), m_vs(std::move(spec)),
      m_si(std::move(si)), m_b_has_succs(b_has_succs),
      m_locality_handle_ref(std::move(locality_handle))
  { }

  void define_type(typer& t)
  {
    t.member(m_handle_ref);
    t.member(m_location);
    t.member(m_tid);
    t.member(m_wf);
    t.member(m_vs);
    t.member(m_si);
    t.member(m_b_has_succs);
    t.member(m_locality_handle_ref);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Unpacks the viewset in group of size one for the tasks.
  ///   Calls @ref detail::create_task to create the task
  ///   object and then immediately executes it.
  ///
  /// @todo Avoid fence below which guards predecessor dataflow setup
  /// traffic.  Options are more use of promises or non accounted rmis.
  ///
  /// @todo Optimize predecessor dataflow setup when a valid paragraph
  ///   pointer can be obtained at destination (i.e., use standard
  ///   setup_flow code path).
  //////////////////////////////////////////////////////////////////////
  void operator()(stapl::executor_base&)
  {
    task_graph* const tg_ptr = resolve_handle<task_graph>(m_handle_ref);

    // nullptr signifies no_succs. deadbeef signifies entry ptr should
    // not be used.  Conditionally set to a valid pointer below.
    detail::edge_entry_base* edge_entry_ptr =
      m_b_has_succs ? (detail::edge_entry_base*) 0xdeadbeef : nullptr;

    // Default to calling processed() locally if there's a local paragraph
    // representative (check an exception to this case in the next condition).
    bool b_call_processed_local = tg_ptr != nullptr;

    // If I have successors (i.e., a producer edge entry to populate),
    // verify that add_producer() was called on this affinity (i.e., this
    // transporter invocation is occurring on same location as add_task
    // which called executor_rmi.  If not, fallback to remote
    // processed() protocol.
    if (tg_ptr && m_b_has_succs)
    {
      detail::edge_entry_base* edge_entry_ptr_tmp =
        tg_ptr->edges().lookup_ptr(m_tid);

      if (edge_entry_ptr_tmp == nullptr
          || !edge_entry_ptr_tmp->local_producer_initialized())
        b_call_processed_local = false;
      else
        edge_entry_ptr = edge_entry_ptr_tmp;
    }

    tg_callback cb = b_call_processed_local ? tg_callback(tg_ptr)
                     : tg_callback(m_handle_ref, m_location, m_tid);

    p_object* const locality_obj_ptr =
      resolve_handle<p_object>(m_locality_handle_ref);

    gang g;

    auto notifier_ptr = new transporter_notifier(cb);

    edge_unpackager edge_unpackager_wf(m_handle_ref, m_location,
                                       notifier_ptr, locality_obj_ptr);

    task_base* const task_ptr =
      detail::create_task<
        SchedulerEntry, std::false_type, std::false_type
      >(nullptr, cb, edge_entry_ptr, std::move(m_si),
        std::move(m_wf),
        edge_unpackager_wf(transporter_unpackager{}(get<Indices>(m_vs)))...);

    notifier_ptr->set_task_ptr(task_ptr);
  }
}; // class transporter


//////////////////////////////////////////////////////////////////////
/// @brief Functor used for setup_flow in nested paragraph directory
/// call to pass to @p invoke_where.  At the location where the nested
/// PARAGRAPH's rmi_handle is known by the directory, use it to initiate
/// a call to request_out_flow on the producer PARAGRAPH.
//////////////////////////////////////////////////////////////////////
template<typename T, typename PortFilter, typename ValueFilter,
         typename LocationMapper>
class setup_flow_func
{
private:
  location_type         m_parent_location;
  std::size_t           m_consumer;
  rmi_handle::reference m_consumer_handle;
  location_type         m_consumer_location;
  PortFilter            m_port_filter;
  ValueFilter           m_value_filter;
  LocationMapper        m_location_mapper;

public:
  setup_flow_func(location_type parent_location,
                  std::size_t consumer,
                  rmi_handle::reference consumer_handle,
                  location_type consumer_location,
                  PortFilter port_filter,
                  ValueFilter value_filter,
                  LocationMapper location_mapper)
    : m_parent_location(parent_location),
      m_consumer(consumer),
      m_consumer_handle(consumer_handle),
      m_consumer_location(consumer_location),
      m_port_filter(std::move(port_filter)),
      m_value_filter(std::move(value_filter)),
      m_location_mapper(std::move(location_mapper))
  { }

  void define_type(typer& t)
  {
    t.member(m_parent_location);
    t.member(m_consumer);
    t.member(m_consumer_handle);
    t.member(m_consumer_location);
    t.member(m_port_filter);
    t.member(m_value_filter);
    t.member(m_location_mapper);
  }

  template<typename Key>
  void operator()(p_object& d, Key const& producer) const
  {
    auto& pg_directory = down_cast<nested_pg_directory&>(d);
    auto  iter         = pg_directory.m_handles.find(producer);

    stapl_assert(iter != pg_directory.m_handles.end(),
                 "failed to find entry");

    rmi_handle::reference producer_handle = iter->second;

    stapl_assert(producer_handle.get_num_locations()
                   == m_consumer_handle.get_num_locations(),
                 "num locations are not equivalent");

    auto* producer_pg = resolve_handle<task_graph>(producer_handle);

    stapl_assert(producer_pg != nullptr,
                 "failed to resolve child producer hande");

    stapl_assert(producer_pg->get_location_id() == get<1>(iter->first),
                 "producer location_id isn't as expected");

    producer_pg->template request_out_flow<T>(
      m_consumer, m_consumer_handle, m_parent_location,
      m_port_filter, m_value_filter, m_location_mapper);
  }
}; // class setup_flow_func


template<typename T, typename PortFilter, typename ValueFilter,
         typename LocationMapper>
void
nested_pg_directory::
setup_flow(location_type consumer_parent_location,
           std::size_t producer,
           std::size_t consumer,
           rmi_handle::reference consumer_handle,
           location_type consumer_location,
           PortFilter port_filter,
           ValueFilter value_filter,
           LocationMapper location_mapper)
{
  using func_t = setup_flow_func<T, PortFilter, ValueFilter, LocationMapper>;

  this->invoke_where(func_t(consumer_parent_location,
                            consumer,
                            consumer_handle,
                            consumer_location,
                            std::move(port_filter),
                            std::move(value_filter),
                            std::move(location_mapper)),
                       key_type(producer, consumer_location));
}


//////////////////////////////////////////////////////////////////////
/// @brief Functor used for setup_parent_flow in nested paragraph directory
/// call to pass to @p invoke_where.  At the location where the nested
/// PARAGRAPH's rmi_handle is known by the directory, use it to initiate
/// a call to request_parent_out_flow on the producer PARAGRAPH.
//////////////////////////////////////////////////////////////////////
template<typename Mapper>
class setup_parent_flow_func
{
private:
  Mapper m_mapper;

public:
  setup_parent_flow_func(Mapper mapper)
    : m_mapper(std::move(mapper))
  { }

  void define_type(typer& t)
  { t.member(m_mapper); }

  template<typename Key>
  void operator()(p_object& d, Key const& producer)
  {
    auto& pg_directory = down_cast<nested_pg_directory&>(d);
    auto  iter         = pg_directory.m_handles.find(producer);

    stapl_assert(iter != pg_directory.m_handles.end(),
                 "failed to find entry");

    rmi_handle::reference producer_handle = iter->second;

    auto* producer_pg = resolve_handle<task_graph>(producer_handle);

    stapl_assert(producer_pg != nullptr,
                 "failed to resolve child producer hande");

    stapl_assert(producer_pg->get_location_id() == 0
                 && get<1>(iter->first) == 0,
                 "producer location_id isn't as expected");

    gang g(*producer_pg);

    using mem_fun_t = void (task_graph::*)(Mapper);

    constexpr mem_fun_t mem_fun =
      &task_graph::request_parent_out_flow<Mapper>;

    async_rmi(all_locations, producer_handle, mem_fun, std::move(m_mapper));
  }
}; // class setup_parent_flow_func


template<typename Mapper>
void
nested_pg_directory::
setup_parent_flow(std::size_t producer, Mapper mapper)
{
  this->invoke_where(
    setup_parent_flow_func<Mapper>(std::move(mapper)),
    key_type(producer, 0));
};


//////////////////////////////////////////////////////////////////////
/// @brief Static functor to check if a PARAGRAPH's scheduler has defined
///   the @p task_placement_all_local type, stating that all tasks created
///   on this location should be run here.  If so, invocation of
///   @p Scheduler::execution_location() is bypassed, with a trivially
///   constructed @ref locality_info object returned.
//////////////////////////////////////////////////////////////////////
template<typename Scheduler,
         bool b_static_all_local =
           has_task_placement_all_local<Scheduler>::value>
struct query_task_placement
{
  template<typename WF, typename... Views>
  static locality_info
  apply(Scheduler& scheduler, task_graph&, WF const& wf, Views const&... views)
  {
    return scheduler.execution_location(wf, views...);
  }
}; // struct query_task_placement


template<typename Scheduler>
struct query_task_placement<Scheduler, true>
{
  template<typename WF, typename... Views>
  static locality_info
  apply(Scheduler& sched, task_graph& tg, WF const&, Views const&...)
  {
    return locality_info(LQ_CERTAIN, get_affinity(),
                         tg.get_rmi_handle(), tg.get_location_id());
  }
}; // struct query_task_placement


template<typename Scheduler>
template<typename WF, typename ExplicitPreds, typename ...Args>
typename std::enable_if<!is_factory<WF>::value, void>::type
task_graph_impl<Scheduler>::
add_task(std::size_t tid, WF const& wf, sched_info_type const& sched_info,
         ExplicitPreds const& pred_list,
         std::size_t num_succs,
         loc_qual qualifier, Args const&... args)
{
  gang g(*this);

  stapl_assert(qualifier != LQ_DONTCARE, "expected LQ_LOOKUP || LQ_CERTAIN");

  boost::optional<locality_info> locality;

  if (qualifier == LQ_LOOKUP)
    locality.reset(query_task_placement<Scheduler>::apply(
      this->current_scheduler(), *this, wf, args...));

  // This task wishes to execute elsewhere (or do LQ_LOOKUP elsewhere).
  // Fire off an async to call add_task on the remote location.
  if (locality
      && locality->qualifier() != LQ_DONTCARE
      && locality->affinity()  != get_affinity())
  {
    const int gang_compare =
      compare_gangs(this->get_rmi_handle(), locality->handle());

    if (gang_compare == 0
        || (gang_compare > 0 && this->get_num_locations() == 1
            && locality->handle().get_num_locations() == 1))
    {
      typedef void (task_graph_impl::* mem_fun_t)
        (std::size_t, WF const&, sched_info_type const&,
         ExplicitPreds const&, std::size_t, loc_qual, Args const&...);

      constexpr mem_fun_t mem_fun =
        &task_graph_impl<Scheduler>::template
          add_task<WF, ExplicitPreds, Args...>;

      const location_type loc =
        this->get_num_locations() == 1 ? 0 : locality->location();

      m_b_no_tasks_escaped = false;

      async_rmi(loc, this->get_rmi_handle(), mem_fun,
                tid, wf, sched_info, pred_list,
                num_succs, locality->qualifier(),
                args...);
      return;
    }
  }

  // Initialize the task on this location.
  this->count_local_task();

  const bool b_has_succs = num_succs != 0;

  using view_set_t =
    tuple<typename subview_type<
      typename paragraph_impl::unwrap_pointer<
        typename std::decay<Args>::type::first_type
      >::type
    >::type...>;

  constexpr size_t num_deferred_localizable =
    detail::count_deferred_localizable<
      typename subview_type<
        typename paragraph_impl::unwrap_pointer<
          typename std::decay<Args>::type::first_type
        >::type
      >::type...
  >::value;

  using edge_t = typename wf_invoke<WF, view_set_t>::result_type;

  // If I have successors, create entry in edge_container to for dataflow.
  detail::edge_entry<edge_t>* const edge_entry_ptr =
    b_has_succs ? &m_edge_ct_ptr->add_producer<edge_t>(tid, num_succs)
    : nullptr;

  if (locality
      && locality->qualifier() != LQ_DONTCARE
      && locality->affinity()  != get_affinity())
  {
    // FIXME - assert here or in dynamic_wf on add_task in nested, migrated task

    stapl_assert(!this->is_persistent(),
      "migrating nested paragraph task when persistent. Not supported yet.");

    stapl_assert(pred_list.size() == 0,
      "migrating nested paragraph task with signal predecessors");

    stapl_assert(locality->qualifier() == LQ_CERTAIN,
      "migrating nested paragraph task without LQ_CERTAIN. Not supported yet.");

    stapl_assert(num_deferred_localizable == 0,
      "migrating task with deferred localizable views. Not supported yet.");

    gang g(this->succs_p_object());

    view_set_t svs(subview_type<
          typename paragraph_impl::unwrap_pointer<
            typename std::decay<Args>::type::first_type>::type
        >::apply(std::move(*(const_cast<Args&>(args).first)),
                 args.second, true)...);

    rmi_fence();
    g.leave();

    auto vs_package = vs_map(transporter_packager(), svs);

    executor_rmi(
      locality->location(),
      locality->handle(),
      transporter<WF, decltype(vs_package), sched_entry_type>
        (this->get_rmi_handle(), this->get_location_id(), tid,
         wf, vs_package, sched_info, b_has_succs, locality->handle()));

    return;
  }

  // Inspect view list to discover number data flow predecessors.
  const size_t num_df_preds =
    pack_ops::functional::plus_<size_t>(0, count_df_preds_func::apply(args)...)
    + num_deferred_localizable;

  const bool b_has_preds = (num_df_preds > 0) || (pred_list.size() > 0);

  // NOTE - this may become more than a simple typedef when we have custom
  // scheduling and/or aggregate consumptions/ etc. Think
  // compute_notifier() metafunction.
  using notifier_base_t = detail::edge_local_notifier_base;
  using notifier_t      = detail::edge_local_notifier;
  using notifier_ptr_t  = notifier_base_t*;

  notifier_ptr_t notifier_ptr = nullptr;

  stapl_assert(m_exec_ptr != nullptr, "attempted to add task to NULL executor");

  // If I have predecessors, setup a edge_local_notifier object with counter for
  // incoming edges to decrement.  Then, notify successors, via edge_view.
  if (b_has_preds)
  {
    const size_t pred_count = num_df_preds + pred_list.size();

    notifier_ptr = new notifier_t(pred_count, this->is_persistent());

    setup_signal(pred_list, m_edge_ct_ptr, notifier_ptr);

    // Call setup flow on non explicitly dependent task
    // (i.e., those specified via consume).
    pack_ops::functional::for_each_(
      setup_edge_flow_func<notifier_ptr_t>(
        notifier_ptr, m_exec_ptr, *m_edge_ct_ptr),
      args...);
  }

  task_type* j;

  {
    gang g(b_has_succs ? succs_p_object() : no_succs_p_object());

    j = detail::create_task<sched_entry_type, migration_t, persistent_t>(
          notifier_ptr, tg_callback(this), edge_entry_ptr, sched_info, wf,
          paragraph_impl::subview_type<
            typename paragraph_impl::unwrap_pointer<
              typename std::decay<Args>::type::first_type>::type
          >::apply(std::move(*(const_cast<Args&>(args).first)),
                   args.second, false)...);
  }

  // Task is runnable, did not alloc notifier, add as start task if persistent.
  if (!b_has_preds)
  {
    this->persister().add_starting_task(j);

    m_exec_ptr->add_task(j);

    return;
  }

  // else...
  //
  // give notifier a pointer to the task.  Either this or subsequent callbacks
  // to it are responsible for giving the executor the task when it's runnable.
  notifier_ptr->set_task_ptr(j, *m_exec_ptr);

} // task_graph::add_task()


////////////////////////////////////////////////////////////////////
/// @brief Type metafunction to compute @ref PARAGRAPH type for
///  a nested PARAGRAPH by expanding the tuple in the task's viewset
///  into a type list to pass to the paragraph's template instantiation.
////////////////////////////////////////////////////////////////////
template<typename Factory, typename Scheduler, typename ViewSet,
         typename IdxList = make_index_sequence<tuple_size<ViewSet>::value>>
struct compute_nested_paragraph_type;


template<typename Factory, typename Scheduler,
         typename ViewSet, std::size_t... Indices>
struct compute_nested_paragraph_type<
  Factory, Scheduler, ViewSet, index_sequence<Indices...>
>
{
  typedef paragraph<
    Scheduler, Factory, typename tuple_element<Indices, ViewSet>::type...
  > type;
};


////////////////////////////////////////////////////////////////////
/// @brief Functor passed to async_construct which is executed after
///   construction of a nested PARAGRAPH (on all locations in the new
///   PARAGRAPH).  Used to initiate discovery for consumer PARAGRAPHs
///   and dataflow from any producers this PARAGRAPH consumes from.
////////////////////////////////////////////////////////////////////
class nested_pg_dataflow_setup
{
public:
  template<typename PG>
  void operator()(PG* pg_ptr) const
  {
    // Store a few properties of child paragraph on stack before starting it,
    // as TD may succeed when we call paragraph::operator().
    const bool b_location_zero = pg_ptr->get_location_id() == 0;
    const auto parent_handle   = pg_ptr->parent_handle();
    const bool b_has_span      = pg_ptr->has_span();

    const bool b_participates_in_dataflow = pg_ptr->participates_in_dataflow();

    pg_ptr->operator()(0, false, false, true);

    // possible paragraph has already been destroyed,
    // after success of termination detection.

    auto* parent_pg = resolve_handle<task_graph>(parent_handle);

    // Keep Location in parent PG from starting its TD until I'm
    // complete.
    // TODO - consider keeping all parents locs alive for duration
    // (at least of child pg's local representative).
    if (b_has_span && b_location_zero)
    {
      stapl_assert(parent_pg != nullptr, "Failed pg handle resolution");
      parent_pg->count_local_task();
    }

    // Don't register if no df predecessors / successors.
    if (!b_participates_in_dataflow)
      return;

    // Make child discoverable to successors
    // Ok to use pg_ptr directly here, since succs couldn't have found me yet,
    // and hence TD cannot have succeeded.
    if (parent_pg != nullptr)
    {
       parent_pg->add_nested_pg(
         make_tuple(pg_ptr->task_id(), pg_ptr->get_location_id()),
         pg_ptr->get_rmi_handle());

       return;
    }

    // else
    //
    add_pg_message(parent_handle,
                   std::bind(&task_graph::add_nested_pg,
                             std::placeholders::_1,
                             make_tuple(pg_ptr->task_id(),
                                        pg_ptr->get_location_id()),
                             pg_ptr->get_rmi_handle()));
  }
}; // class nested_pg_dataflow_setup


//////////////////////////////////////////////////////////////////////
/// @brief Static functor with partial specialization to detect if the
/// factory for a nested PARAGRAPH provides a custom scheduler for the
/// PARAGRAPH or if the default_scheduler should be used.
//////////////////////////////////////////////////////////////////////
template<typename Factory, bool = has_scheduler_type<Factory>::value>
struct get_nested_scheduler
{
  using type = default_scheduler;

  static type apply(Factory const& factory)
  { return type(); }
};


template<typename Factory>
struct get_nested_scheduler<Factory, true>
{
  using type = typename Factory::scheduler_type;

  static type apply(Factory const& factory)
  { return factory.get_scheduler(); }
};


////////////////////////////////////////////////////////////////////
/// @brief Facilitates creation of one-sided PARAGRAPH creation via
///   construct.  Template exist instead of inline in @p add_task
///   to allow creation of index integer pack via partial specialization
///   so that we can expand the viewset into an argument list an apply
///   view packaging prior to transmission.
////////////////////////////////////////////////////////////////////
template<typename IdxList>
struct create_paragraph;


template<std::size_t ...Indices>
struct create_paragraph<index_sequence<Indices...>>
{
  using handle_t = rmi_handle::reference;

  //////////////////////////////////////////////////////////////////////
  /// @brief Calls construct to create a paragraph and registers a callback
  ///   via @ref future::async_then() to execute it once initialization has
  ///   succeeded.
  ///
  /// Ephemeral parent @ref paragraph signature. Child paragraph uses
  /// @ref default_scheduler.
  ///
  /// @param parent Reference to the parent paragraph.
  /// @param comm_group_handle The handle defining the group where the
  ///   subsequent enumeration of locations are defined.
  /// @param locs The set of locations in an existing communication group
  ///   where this paragraph should be initialized.
  /// @param tid The task identifier of the nested paragraph in the
  ///   parent paragraph.
  /// @param f The factory for the nested paragraph.
  /// @param num_succs Number of successors of this nested paragraph task
  ///   within the parent paragraph.
  /// @param b_has_preds Boolean denoting whether nested paragraph task
  ///   has intra-predecessors within the parent paragraph.
  /// @param viewset The set of views the nested paragraph operates on.
  //////////////////////////////////////////////////////////////////////
  template<typename ParentPG, typename Factory, typename ViewSet>
  static void
  apply(ParentPG& parent,
        std::false_type,
        handle_t const& comm_group_handle,
        std::vector<location_type> const& locs,
        std::size_t tid,
        Factory const& f,
        std::size_t num_succs,
        ViewSet& viewset)
  {
    using scheduler_t = typename get_nested_scheduler<Factory>::type;

    using paragraph_t =
      typename compute_nested_paragraph_type<
        Factory, scheduler_t, ViewSet>::type;

    // If location list is empty, densely spawn on all locations of specified
    // handle.  Use more efficient signature of construct().
    //
    // For ephemeral nested paragraphs, the paragraph will manage its own
    // lifetime based on success of termination detection.
    //
    if (locs.empty())
    {
      // Densely spawn on all locations of specified handle.
      async_construct<paragraph_t>(
        nested_pg_dataflow_setup(),
        comm_group_handle, all_locations, nested_tag(), f,
        get_nested_scheduler<Factory>::apply(f),
        false,
        parent.get_rmi_handle(),
        parent.get_location_id(),
        parent.has_only_local_tasks(),
        tid, num_succs,
        transporter_packager()(get<Indices>(viewset))...);
    }
    else
    {
      async_construct<paragraph_t>(
        nested_pg_dataflow_setup(),
        comm_group_handle, location_range(locs), nested_tag(), f,
        get_nested_scheduler<Factory>::apply(f),
        false,
        parent.get_rmi_handle(),
        parent.get_location_id(),
        parent.has_only_local_tasks(),
        tid, num_succs,
        transporter_packager()(get<Indices>(viewset))...);
    }
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Calls construct to create a paragraph and registers a callback
  ///   via @ref future::async_then() to execute it once initialization has
  ///   succeeded.
  ///
  /// Persistent parent @ref paragraph signature. Child paragraph uses
  ///  @ref persistent_scheduler.
  /// Hold paragraph in @ref nested_pg_task and add to list of starting tasks.
  ///
  /// @param parent Reference to the parent paragraph.
  /// @param comm_group_handle The handle defining the group where the
  ///   subsequent enumeration of locations are defined.
  /// @param locs The set of locations in an existing communication group
  ///   where this paragraph should be initialized.
  /// @param tid The task identifier of the nested paragraph in the
  ///   parent paragraph.
  /// @param f The factory for the nested paragraph.
  /// @param num_succs Number of successors of this nested paragraph task
  ///   within the parent paragraph.
  /// @param b_has_preds Boolean denoting whether nested paragraph task
  ///   has intra-predecessors within the parent paragraph.
  /// @param viewset The set of views the nested paragraph operates on.
  //////////////////////////////////////////////////////////////////////
  template<typename ParentPG, typename Factory, typename ViewSet>
  static void
  apply(ParentPG& parent,
        std::true_type,
        handle_t const& comm_group_handle,
        std::vector<location_type> const& locs,
        std::size_t tid,
        Factory const& f,
        std::size_t num_succs,
        ViewSet& viewset)
  {
    // do we want to refine to detect non df preds?
    // stapl_assert(!b_has_preds, "Preds for Persistent PGs not implemented");

    using paragraph_t =
      typename compute_nested_paragraph_type<
        Factory, persistent_scheduler, ViewSet>::type;

    promise<rmi_handle::reference> p;

    auto fut = p.get_future();

    auto dataflow_setup_func =
      std::bind(
        [](paragraph_t* pg_ptr,
           promise<rmi_handle::reference> p)
        {
          const bool b_participates_in_dataflow =
            pg_ptr->participates_in_dataflow();

          // Start first execution of paragraph.  PG will live past this
          // first call, as promise below must be set to allow it to be
          // destroyed in persistent mode.
          pg_ptr->operator()(0, false, false, true);

          // Inform parent of handle so that it can reinvoke
          location_type loc = pg_ptr->get_location_id();

          if (loc == 0)
            p.set_value(pg_ptr->get_rmi_handle());

          auto* parent_pg = resolve_handle<task_graph>(pg_ptr->parent_handle());

          /// @todo Expand to use RMIs if not resolvable..
          stapl_assert(parent_pg != nullptr, "Failed pg handle resolution");

          if (pg_ptr->has_span() && loc == 0)
            parent_pg->count_local_task();

          // Don't register if no successors.  Also, possible paragraph has
          // already been destroyed, after success of termination detection.
          if (b_participates_in_dataflow)
          {
            // Keep Location in parent PG from starting its TD until I'm
            // complete.
            // Make myself discoverable to successors
            parent_pg->add_nested_pg(
              make_tuple(pg_ptr->task_id(), pg_ptr->get_location_id()),
              pg_ptr->get_rmi_handle());
          }
        },
        std::placeholders::_1,
        std::move(p)
      );

    // If location list is empty, densely spawn on all locations of specified
    // handle.  Use more efficient signature of construct().
    if (locs.empty())
    {
      // Densely spawn on all locations of specified handle.
      async_construct<paragraph_t>(
        dataflow_setup_func,
        comm_group_handle, all_locations, nested_tag(), f,
        persistent_scheduler(), false,
        parent.get_rmi_handle(),
        parent.get_location_id(),
        parent.has_only_local_tasks(),
        tid, num_succs,
        transporter_packager()(get<Indices>(viewset))...);
    }
    else
    {
      async_construct<paragraph_t>(
        dataflow_setup_func,
        comm_group_handle, location_range(locs), nested_tag(), f,
        persistent_scheduler(), false,
        parent.get_rmi_handle(),
        parent.get_location_id(),
        parent.has_only_local_tasks(),
        tid, num_succs,
        transporter_packager()(get<Indices>(viewset))...);
    }

    fut.async_then(
      [&parent](future<rmi_handle::reference> f)
      {
        rmi_handle::reference handle = f.get();

        using base_task_t   = typename ParentPG::task_type;
        using sched_entry_t = typename ParentPG::sched_entry_type;
        using task_t        = nested_pg_task<sched_entry_t, paragraph_t>;
        using sched_info_t  = typename ParentPG::sched_info_type;

        base_task_t* task_ptr = new task_t(handle, sched_info_t());

        parent.add_starting_task(task_ptr);
      });
  }
};


template<typename Scheduler>
template<typename Factory, typename ExplicitPreds, typename ...Args>
typename std::enable_if<is_factory<Factory>::value, void>::type
task_graph_impl<Scheduler>::
add_task(std::size_t tid, Factory const& f, sched_info_type const& sched_info,
         ExplicitPreds const& pred_list, std::size_t num_succs,
         loc_qual qualifier, Args const&... args)
{
  gang g(*this);

  stapl_assert(pred_list.size() == 0,
    "Explicit predecessors for nested pg not supported");

  using view_set_t =
    tuple<
      typename std::remove_reference<
        typename subview_type<
          typename paragraph_impl::unwrap_pointer<
            typename std::decay<Args>::type::first_type
          >::type
        >::type
      >::type...>;

  using edge_t = typename wf_invoke<Factory, view_set_t>::result_type;

  // If I have successors, create entry in edge_container to for dataflow.
  if (!need_result_view<Factory, view_set_t>::value && num_succs > 0)
    m_edge_ct_ptr->add_producer<edge_t>(tid, num_succs);

  auto loc_pair = this->current_scheduler().execution_location(f, args...);

  const rmi_handle::reference comm_group_handle =
    loc_pair.first == rmi_handle::reference()
    ? this->get_rmi_handle() : loc_pair.first;

  gang g2;
    view_set_t svs(paragraph_impl::subview_type<
          typename paragraph_impl::unwrap_pointer<
            typename std::decay<Args>::type::first_type>::type
        >::apply(std::move(*(const_cast<Args&>(args).first)),
                 args.second, true)...);
    rmi_fence();
  g2.leave();

  //
  m_b_no_tasks_escaped = false;

  if (!need_result_view<Factory, view_set_t>::value)
    this->count_local_task();

  create_paragraph<make_index_sequence<sizeof...(Args)>>::apply(
    *this, persistent_t(),
    comm_group_handle, loc_pair.second, tid,
    f, num_succs, svs
  );
} // task_graph::add_task(Factory)


template<typename SchedulerEntry, typename WF, typename ViewSet,
         std::size_t ...Indices>
void task_graph::
add_migrated_task(std::size_t tid, size_t unknown_consumer_cnt,
                  typename SchedulerEntry::sched_info_type const& sched_info,
                  WF const& wf, ViewSet const& vs,
                  index_sequence<Indices...> const&)
{
  ++m_local_added_count;

  stapl_assert(!this->is_persistent(), "Persistent / Migrate not together yet");

  typedef tuple<
            typename migration_unpacker<
              typename tuple_element<Indices, ViewSet>::type
            >::type...
          > packed_vs_t;

  typedef tuple<decltype(
            transporter_unpackager{}(get<Indices>(std::declval<packed_vs_t>()))
          )...> vs_t;
#ifndef STAPL_NDEBUG
  constexpr size_t num_deferred_localizable =
    detail::count_deferred_localizable<
      decltype(
        transporter_unpackager{}(get<Indices>(std::declval<packed_vs_t>())))...
    >::value;
#endif

   stapl_assert(num_deferred_localizable == 0,
     "migrated task with deferred localizable views. Not Supported.");

  typedef typename wf_invoke<WF, vs_t>::result_type edge_t;

  const bool b_has_succs = unknown_consumer_cnt != 0;

  // If I have successors, create entry in edge_container to for dataflow.
  detail::edge_entry<edge_t>* const edge_entry_ptr =
    b_has_succs ?
      &m_edge_ct_ptr->add_producer<edge_t>(tid, unknown_consumer_cnt, true)
    : nullptr;

  task_base* j;

  {
    gang g(b_has_succs ? succs_p_object() : no_succs_p_object());

    packed_vs_t packed_vs{
      migration_unpacker<
        typename tuple_element<Indices, ViewSet>::type
      >::apply(get<Indices>(vs), *m_edge_ct_ptr)...};

    j = detail::create_task<
          SchedulerEntry, std::true_type, std::false_type
        >(nullptr, tg_callback(this), edge_entry_ptr, sched_info, wf,
          transporter_unpackager{}(get<Indices>(packed_vs))...);

    vs_map(lazy_ref_release_func(tg_callback(this)), packed_vs);
  }

  m_exec_ptr->add_task(j);

} // task_graph::add_migrated_task()

} // namespace paragraph_impl

using paragraph_impl::task_graph;

} // namespace stapl

#include <stapl/paragraph/edge_container/edge_container.hpp>
#include <stapl/paragraph/tasks/task_creation.hpp>

#endif // STAPL_PARAGRAPH_PARAGRAPH_IMPL_HPP
