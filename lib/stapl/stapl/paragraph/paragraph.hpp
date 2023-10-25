/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_PARAGRAPH_HPP
#define STAPL_PARAGRAPH_PARAGRAPH_HPP

#include <utility>
#include <algorithm>
#include <type_traits>

#include <boost/function.hpp>
#include <boost/optional.hpp>
#include <boost/utility/typed_in_place_factory.hpp>

#include <stapl/paragraph/paragraph_fwd.h>
#include <stapl/paragraph/wf_invoke.hpp>
#include <stapl/paragraph/paragraph_impl.hpp>
#include <stapl/skeletons/explicit/task_graph_factories.h>
#include <stapl/views/metadata/coarseners/null.hpp>
#include <stapl/paragraph/view_operations/view_manager.hpp>
#include <stapl/paragraph/edge_container/views/edge_view.hpp>

#include <stapl/paragraph/make_paragraph.hpp>
#include <stapl/paragraph/tasks/task.hpp>
#include <stapl/skeletons/utility/dynamic_wf.hpp>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>

#include <stapl/runtime.hpp>
#include <stapl/runtime/executor/terminator.hpp>

#include <stapl/utility/tuple.hpp>

#include <stapl/paragraph/tg_result_view.hpp>

namespace stapl {


//////////////////////////////////////////////////////////////////////
/// @brief Get a reference to this thread's pair of queues of messages
/// sent to task graphs and associated edge containers before their
/// construction.
//////////////////////////////////////////////////////////////////////
tuple<paragraph_messages::tg_buffer_t&,
      paragraph_messages::ec_buffer_t&>
get_pg_message_queues(void);


//////////////////////////////////////////////////////////////////////
/// @brief Helper functor used by @ref check_pg_messages to check if
/// message is for a given input view if it is a paragraph port dataflow
/// view.  If it is, call the message and return true, otherwise
/// return false.
//////////////////////////////////////////////////////////////////////
struct pg_message_handler
{
private:

  static bool
  impl(edge_container& ec, paragraph_messages::ec_msg_t const& msg)
  {
    if (ec.get_rmi_handle() == get<0>(msg))
    {
      get<1>(msg)(ec);
      return true;
    }

    return false;
  }

public:
  template<typename View>
  static bool apply(View const&, paragraph_messages::ec_msg_t const&)
  { return false; }

  template<typename T, typename ...OptionalParams>
  static bool
  apply(nested_pg_view_subview<T, OptionalParams...> const& v,
        paragraph_messages::ec_msg_t const& msg)
  { return impl(v.container(), msg); }

  template<typename T, typename ...OptionalParams>
  static bool
  apply(nested_parent_pg_view_subview<T, OptionalParams...> const& v,
        paragraph_messages::ec_msg_t const& msg)
  { return impl(v.container(), msg); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Static functor that returns true if the input parameter's
/// type is that of an input, dataflow view.
//////////////////////////////////////////////////////////////////////
struct check_nested_df_preds
{
public:
  template<typename View>
  static constexpr bool apply(View const&)
  { return false; }

  template<typename T, typename ...OptionalParams>
  static constexpr bool
  apply(nested_pg_view_subview<T, OptionalParams...> const&)
  { return true; }

  template<typename T, typename ...OptionalParams>
  static constexpr bool
  apply(nested_parent_pg_view_subview<T, OptionalParams...> const&)
  { return true; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Returns true if any of the views in the given tuple are
/// dataflow input views from other paragraphs.
//////////////////////////////////////////////////////////////////////
template<typename... Views, std::size_t... Indices>
constexpr bool has_nested_df_preds(tuple<Views...> const& views,
                                   index_sequence<Indices...> const&)
{
  return pack_ops::functional::or_(
    false, check_nested_df_preds::apply(get<Indices>(views))...);
}


//////////////////////////////////////////////////////////////////////
/// @brief Call in paragraph constructor to see if messages from affinity
/// colocated child paragraphs have queued messages for it (i.e., messages
/// child wants the parent to forward for it).
//////////////////////////////////////////////////////////////////////
template<typename... Views, std::size_t... Indices>
void check_pg_messages(paragraph_impl::task_graph& tg,
                       tuple<Views...>& views,
                       index_sequence<Indices...> const&)
{
  auto queues  = get_pg_message_queues();
  auto& tg_vec = get<0>(queues);
  auto& ec_vec = get<1>(queues);

  // Check messages sent to task_graph
  tg_vec.erase(
    std::remove_if(tg_vec.begin(), tg_vec.end(),
                   [&tg](paragraph_messages::tg_msg_t const& msg) -> bool
                   {
                     if (tg.get_rmi_handle() == get<0>(msg))
                     {
                       get<1>(msg)(tg);
                       return true;
                     }

                     return false;
                   }),
    tg_vec.end());

  // Check messages send to input ports (edge_container instances).
  ec_vec.erase(
    std::remove_if(ec_vec.begin(), ec_vec.end(),
                   [&views](paragraph_messages::ec_msg_t const& msg) -> bool
                   {
                     return pack_ops::functional::or_(
                       false,
                        pg_message_handler::apply(get<Indices>(views), msg)...);
                   }),
    ec_vec.end());
}


namespace detail {


//////////////////////////////////////////////////////////////////////
/// @brief Use RAII to conditionally invoke a executor drain after the body of
///   @ref paragraph::operator().
/// @ingroup paragraph
//////////////////////////////////////////////////////////////////////
struct scoped_drain_call
{
  /// @brief Boolean guards drain call in destructor.
  bool m_b_drain;

  scoped_drain_call(bool b_drain)
    : m_b_drain(b_drain)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Empty function call used in paragraph to avoid compiler warning.
  //////////////////////////////////////////////////////////////////////
  void foo()
  { }

  ~scoped_drain_call()
  {
    if (m_b_drain)
      get_executor()(execute_all);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Reduction operator in termination detection.  Add task
/// added / processed counts and AND escape tasks booleans (used
/// for early termination optimization).
//////////////////////////////////////////////////////////////////////
struct termination_reduce
{
  using value_type = std::pair<int, bool>;

  value_type operator()(value_type const& lhs, value_type const& rhs) const
  {
    return value_type(lhs.first + rhs.first, lhs.second && rhs.second);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Responsible for initializing the termination detection object that
///   the PARAGRAPH's executor employs to determine when the local computation
///   for the PARAGRAPH is completed.
/// @ingroup paragraph
///
/// Partial specialization of this class provides a mechanism to customize
/// the termination detection algorithm of a given PARAGRAPH.
///
/// @tparam Factory The factory type of the associated PARAGRAPH.  Unused in the
/// class definition, but allows class specialization based on this type.
//////////////////////////////////////////////////////////////////////
template<typename FactoryTag>
struct terminator_initializer
{
  using is_default = void;

  /// @brief The type of the terminator object.  By default, uses
  /// @ref terminator, defined in the runtime.
  using terminator_t = terminator<std::pair<int, bool>, termination_reduce>;

  //////////////////////////////////////////////////////////////////////
  /// @brief Signature function operator that creates a termination
  ///   detection object in the paragraph constructor.
  ///
  /// @param tg The associated PARAGRAPH instance.
  /// @param svs The post data coarsening input views of the PARAGRAPH.
  /// @param t Uninitialized terminator stored in paragraph.  Initialized
  /// in this call as desired.
  //////////////////////////////////////////////////////////////////////
  template<typename SVS>
  void
  operator()(task_graph& tg, SVS& svs, boost::optional<terminator_t>& t) const
  {
    t = boost::in_place<terminator_t>(
          termination_reduce(),
          [&tg] { return tg.termination_value(); },
          tg.has_only_local_tasks()
        );
  }
}; // struct terminator_initializer

} // namespace detail


namespace paragraph_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor used by @ref paragraph to reset port edges from producing
/// paragraph prior to re-invocation of the paragraph in persistent mode.
//////////////////////////////////////////////////////////////////////
struct in_edge_reset
{
  using result_type = void;

  template<typename T>
  void operator()(T const&) const
  { }

  template<typename T, typename ...OptionalParams>
  void
  operator()(
  nested_pg_view_subview<T, OptionalParams...> const& v) const
  {
    v.edges().reset_entry_values();
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction that computes the coarsener type for a given
/// @c Factory.
/// @ingroup pgUtility
//////////////////////////////////////////////////////////////////////
template <typename Factory, bool stat = has_coarsener_type<Factory>::value>
struct coarsener_type {
  using type = typename Factory::coarsener_type;

  static type call(Factory const& factory) { return factory.get_coarsener(); }
};

//////////////////////////////////////////////////////////////////////
/// @brief The specialization of coarsener_type metafunction for the
/// case that the coarsener_type is not defined by the @c Factory.
/// @ingroup pgUtility
//////////////////////////////////////////////////////////////////////
template <typename Factory>
struct coarsener_type<Factory, false> {
  using type = null_coarsener;

  static type call(Factory const&) { return type(); }
};

//////////////////////////////////////////////////////////////////////
/// @brief Trivial holder for an object (currently the factory in the
/// @ref paragraph which holds an instance and can be inherited from
/// without polluting the derived class with members which may cause
/// naming conflicts.
///
/// The factory needs inheritance so that it can be initialized before
/// other base classes (not possible if it's a data member).
//////////////////////////////////////////////////////////////////////
template<typename T>
class object_wrapper
{
private:
  T m_t;

public:
  template<typename ...Args>
  object_wrapper(Args&&... args)
    : m_t(std::forward<Args>(args)...)
  { }

  T& get(void)
  { return m_t; };

  T const& get(void) const
  { return m_t; };
};


//////////////////////////////////////////////////////////////////////
/// @brief The PARAGRAPH is the dependence graph
///   representation of programs in STAPL.  It's generally not advised for users
///   to instantiate it directly, but rather use @ref stapl::make_paragraph.
/// @ingroup paragraph
///
/// @tparam Scheduler The scheduler this PARAGRAPH passes to the executor that
///   is responsible for ordering runnable tasks for execution.  It may also
///   request migration of runnable tasks, to support policies such as dynamic,
///   cross location task load balancing.
///
/// @tparam Factory The task factory for this PARAGRAPH which is responsible for
///   populating the PARAGRAPH with tasks and edges.  One instance of the
///   factory will be inserted into the executor on each location of the
///   PARAGRAPH during the first invocation (subsequent calls on a persistent
///   PARAGRAPH avoid this operation).
///
/// @tparam ... A list of views that the PARAGRAPH should provide to the factory
///   when it is invoked.  The factory may provide a data coarsening method to
///   be applied prior to factory invocation.
///
/// Most of the implementation is located in untyped (or less typed) base
/// classes to avoid code bloat. This derived class exists mainly to hold
/// data members needing the full PARAGRAPH type (i.e., views, factory, and
/// scheduler) and to implement several virtual functions providing an
/// interface to these members. The primary public operator is the function
/// operator (to those creating a PARAGRAPH, it appears as function object).
/// It also has interfaces for result specification by the factory.
///
/// @todo There are two copies of the post-coarsened views kept, one in the
///   factory task and one as a member of this class.  The former is probably
///   better, provide a method (@ref task_graph_impl has pointer to the
///   factory task) to access it.
///
/// @todo Further template instantiation reduction can probably be had by making
///  the PARAGRAPH a simple class instead of a class template.  This requires:
///    (1) make this struct result
///    (2) view template operator()
///    (3) defer coarsening
///    (4) make edge_view creation disappear into task_graph
//////////////////////////////////////////////////////////////////////
template<typename Scheduler, typename Factory, typename... Views>
class paragraph final
  : public object_wrapper<
      factory_task<
        Factory,
        typename view_manager<
          typename coarsener_type<Factory>::type,
          Views...
        >::result_type,
        typename Scheduler::entry_type,
        paragraph_impl::paragraph_view<Scheduler>
      >
    >,
    public task_graph_impl<Scheduler>,
    public detail::result_manager<
      typename wf_invoke<
        Factory,
        typename view_manager<
          typename coarsener_type<Factory>::type,
          Views...
        >::result_type
      >::result_type,
      need_result_view<Factory, tuple<Views...>>::value
    >
{
private:
  using coarsener_t     = typename coarsener_type<Factory>::type;

  using view_manager_t  = view_manager<coarsener_t, Views...>;

  // Post Coarsening ViewSet type
  using viewset_t         = typename view_manager_t::result_type;

  using factory_task_type =
    factory_task<Factory, viewset_t,
                 typename Scheduler::entry_type,
                  paragraph_impl::paragraph_view<Scheduler>>;

  using factory_base_type = object_wrapper<factory_task_type>;

  using result_manager_t  =
    detail::result_manager<
      typename wf_invoke<Factory, viewset_t>::result_type,
      need_result_view<Factory, tuple<Views...>>::value
    >;

  using terminator_init_t =
    typename detail::terminator_initializer<typename tag_type<Factory>::type>;

  using terminator_t      = typename terminator_init_t::terminator_t;

  /// @brief The termination detection object used by this PARAGRAPH.
  /// Stored as a pointer so that it can be destroyed and
  /// reinitialized for persistent PARAGRAPH invocations.
  boost::optional<terminator_t>                             m_terminator;

  detail::result_container_base& get_result_container(void) final
  {
    return *result_manager_t::get_result_container();
  }

  bool has_outstanding_consumers(void) const final
  {
    return result_manager_t::has_outstanding_consumers();
  }

  void reset_in_out_containers(void) final
  {
    // Call reset on the edge_container underneath inter-paragraph input views.
    vs_map(in_edge_reset(), get_factory_task().views());

    // Reset the output container
    result_manager_t::reset();
  }

  factory_task_type&
  get_factory_task(void) final
  {
    return factory_base_type::get();
  }

  factory_task_type const&
  get_factory_task(void) const final
  {
    return factory_base_type::get();
  }

public:
  using result_type = typename result_manager_t::result_type;

  //////////////////////////////////////////////////////////////////////
  /// @todo @ref p_object::advance_epoch() call may not be needed anymore. The
  /// one present in operator() prior to invoking the factory is probably
  /// sufficient.
  /// However, maybe it makes sense to have a fully constructed object before
  /// proceeding.  Evaluate and remove the call or this todo.
  //////////////////////////////////////////////////////////////////////
  paragraph(paragraph const& other)
    : factory_base_type(other),
      task_graph_impl<Scheduler>(other),
      m_terminator(other.m_terminator)
  {
    this->advance_epoch();
    check_pg_messages(*this, get_factory_task().views(),
                      make_index_sequence<sizeof...(Views)>());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor called to initialize a new PARAGRAPH collectively.
  ///
  /// @param factory The factory for this PARAGRAPH.
  /// @param scheduler The scheduler to use with this PARAGRAPH.
  /// @param enable_migration Boolean denoting whether migration support should
  ///   be enabled in this PARAGRAPH instantiation.
  /// @param parent_handle_ref Handle to the parent paragraph of this paragraph.
  /// @param parent_location The location in the parent paragraph responsible
  ///  for metadata about this paragraph.
  /// @param b_parent_notify_all_locations Flags whether this paragraph should
  ///  flag all colocated locations in the parent when it finishes.
  /// @param task_id The task identifier in the parent paragraph for this
  ///   paragraph.
  /// @param b_has_succs Boolean denoting whether paragraph
  ///   has intra-paragraph successors within the parent paragraph.
  /// @param ... Variadic list of views to pass to @p factory invocation.
  ///
  /// @todo @ref p_object::advance_epoch() call may not be needed anymore. The
  /// one present in operator() prior to invoking the factory is probably
  /// sufficient. However, maybe it makes sense to have a fully constructed
  /// object before proceeding.  Evaluate and remove the call or this todo.
  //////////////////////////////////////////////////////////////////////
  paragraph(Factory factory,
            Views const&... views,
            Scheduler scheduler = Scheduler(),
            const bool enable_migration = false,
            rmi_handle::reference parent_handle_ref = rmi_handle::reference(),
            location_type parent_location = invalid_location_id,
            bool b_parent_notify_all_locations = false,
            size_t task_id                = 0,
            std::size_t num_succs         = 0)
    : factory_base_type(
        std::move(factory),
        view_manager_t::apply(
          coarsener_type<Factory>::call(factory), views...)),
      task_graph_impl<Scheduler>(
        compute_tid_mapper_type<Factory, sizeof...(Views)>::apply(
         factory_base_type::get().factory(), factory_base_type::get().views()),
        &factory_base_type::get(),
        std::move(scheduler),
        enable_migration,
        num_succs,
        num_succs > 0
          || has_nested_df_preds(
               get_factory_task().views(),
               make_index_sequence<sizeof...(Views)>()),
        std::move(parent_handle_ref), parent_location,
        b_parent_notify_all_locations, task_id)
  {
    terminator_init_t()(*this, get_factory_task().views(), m_terminator);

    this->advance_epoch();
    check_pg_messages(*this, get_factory_task().views(),
                      make_index_sequence<sizeof...(Views)>());
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor for one-sided initialization of a new PARAGRAPH.
  ///
  /// @param ignored Tag dispatch to select this constructor in one-sided
  ///  spawning process.
  /// @param factory The factory for this PARAGRAPH.
  ///  from skeleton parameter, as the former is not serializable.
  /// @param scheduler The scheduler to use with this PARAGRAPH.
  /// @param enable_migration Boolean denoting whether migration support should
  ///   be enabled in this PARAGRAPH instantiation.
  /// @param parent_handle_ref Handle to the parent paragraph of this paragraph.
  /// @param parent_location The location in the parent paragraph responsible
  ///  for metadata about this paragraph.
  /// @param b_parent_notify_all_locations Flags whether this paragraph should
  ///  flag all colocated locations in the parent when it finishes.
  /// @param task_id The task identifier in the parent paragraph for this
  ///   paragraph.
  /// @param b_has_succs Boolean denoting whether paragraph
  ///   has intra-paragraph successors within the parent paragraph.
  /// @param packaged_views Set of packaged views which are unpackaged,
  ///   coarsened, and then passed to @p factory invocation.
  //////////////////////////////////////////////////////////////////////
  template<typename ...PackagedViews>
  paragraph(nested_tag const& ignored,
            Factory factory,
            Scheduler const& scheduler,
            const bool enable_migration,
            rmi_handle::reference parent_handle_ref,
            location_type parent_location,
            bool b_parent_notify_all_locations,
            size_t task_id,
            std::size_t num_succs,
            PackagedViews /*const*/&&... packaged_views)
    : factory_base_type(
        std::move(factory),
        view_manager_t::apply(
          coarsener_type<Factory>::call(factory),
          transporter_unpackager()(packaged_views)...)),
      task_graph_impl<Scheduler>(
        compute_tid_mapper_type<Factory, sizeof...(Views)>::apply(
         factory_base_type::get().factory(), factory_base_type::get().views()),
        &factory_base_type::get(),
        std::move(scheduler),
        enable_migration,
        num_succs,
        num_succs > 0
          || has_nested_df_preds(
               get_factory_task().views(),
               make_index_sequence<sizeof...(Views)>()),
        std::move(parent_handle_ref), parent_location,
        b_parent_notify_all_locations, task_id)
  {
    vs_map(pg_edge_unpackager(*this), get_factory_task().views());

    terminator_init_t()(*this, get_factory_task().views(), m_terminator);

    this->advance_epoch();
    check_pg_messages(*this, get_factory_task().views(),
                      make_index_sequence<sizeof...(Views)>());
  }


  bool has_span(void) const final
  {
    return need_result_view<Factory, tuple<Views...>>::value;
  }


  void pulse_terminator(void) final
  {
    if (this->hold_termination())
      return;

    gang g(*this);
    (*m_terminator)();
  }


  void set_result(std::size_t index, task_graph::task_id_t task_id) final
  {
    this->set_result_impl(index, task_id, *this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Method that starts population and execution of a PARAGRAPH.
  ///
  /// @param unused Unused parameter that provides unique signature to function
  ///   that enables non-blocking behavior by default.
  ///
  /// @param blocking True if blocking behavior of PARAGRAPH (i.e., heap
  ///   allocate structures, etc) should be enabled.
  ///
  /// @param force_evaluation When in non-blocking mode, still enforce that the
  ///   the function operator doesn't return until termination detection
  ///   succeeds and the value is available.
  ///
  /// @param force_evaluation When in non-blocking mode, still enforce that the
  ///   the function operator doesn't return until termination detection
  ///   succeeds and the value is available.
  ///
  /// @param one_sided When true, this paragraph was created via a one-sided,
  ///   nested parallelism invocation.  Enforce non-blocking semantics, and make
  ///   sure the terminator was initialized in the paragraph constructor.
  ///
  /// @return Type is defined by @ref result_manager, but in general is a
  ///   proxy over the value returned by the PARAGRAPH on this location
  ///   (defined by factory via call to @ref paragraph_view::set_result).
  ///
  /// @todo Reevaluate whether we need two signatures with the current
  ///   parameters or if we can simplify the calling convention.
  ///
  /// @todo Verify that @ref p_object::advance_epoch() is still needed. Maybe
  ///   required for PARAGRAPH call stack push, but for non blocking, the call
  ///   stack isn't guaranteed to be the same anyways.  Related to check for
  ///   same in paragraph constructor.
  ///
  /// @todo The parameters blocking and force_evaluation are close to
  ///   being redundant now, now that the internal implementation more or less
  ///   assumes non-blocking by default now.  Verify there's no vestiges of the
  ///   blocking implementation and remove the second parameter.
  //////////////////////////////////////////////////////////////////////
  result_type operator()(int  unused,
                         bool blocking         = false,
                         bool force_evaluation = false,
                         bool one_sided        = false)
  {
    //
    // Switch to gang of this paragraph so that traffic is properly accounted
    // for, and invoke base class operator for template independent
    // initialization code.
    //
    gang g(*this);

    task_graph::operator()(unused, blocking, force_evaluation, one_sided);

    //
    // Setup result storage and define interaction with termination detection.
    //
    typename result_manager_t::tmp_result_type result =
      result_manager_t::apply(*this, one_sided);

    using os_td_wait_t = typename result_manager_t::os_td_wait_t;

    os_td_wait_t* os_td_wait_ptr = nullptr;

    if (one_sided)
      os_td_wait_ptr = this->set_os_result_notifier(*this, result);

    this->advance_epoch();


    //
    // Instantiate executor and setup interaction with terminator.
    //
    auto executor_ptr =
      new executor<task_graph_impl<Scheduler>>{*this, this->passed_scheduler()};

    this->set_executor(executor_ptr);

    m_terminator->set_notifier(
      [this, executor_ptr, os_td_wait_ptr](void)
      {
        // Put a copy of these lambda captured pointers on the
        // stack so that they can be used after nofiy_finished()
        // in the executor deletes the terminator holding this
        // lambda instance in a function wrapper.
        os_td_wait_t* stack_os_td_wait_ptr = os_td_wait_ptr;
        task_graph*   stack_tg_ptr         = this;

        executor_ptr->notify_finished();
        stack_tg_ptr->notify_termination();

        // Once this paragraph has been completely destroyed, signal
        // notifier responsible with informing parent that TD has
        // succeeded.
        if (stack_os_td_wait_ptr != nullptr)
          stack_os_td_wait_ptr->notify_td();
      }
    );


    //
    // Provide executor with runnable tasks.
    //
    const auto task_range = this->persister().get_executor_start_tasks();

    for (auto** it = task_range.first; it != task_range.second; ++it)
    {
      if ((*it)->is_nested_pg_task())
      {
        // Switch into a single location which will be fenced prior to the
        // pg TD success, so the coherence traffic of the
        // async_rmi(all_locations) which fires the task is properly received.
        gang(this->no_succs_p_object());

        (*it)->operator()(this);
      }
      else
        executor_ptr->add_task(*it);
    }


    //
    // Either force blocking execution or hook this executor into the current
    // gang_executor before returning view or reference to result.
    //
    const bool b_is_nested = this->get_rmi_handle().get_gang_id() != 0;

    // PARAGRAPH may possibly be destroyed in this call.
    // Do Not access data members after this call.
    if (this->get_location_md().get_gang_md().get_id() == 0 || one_sided) {
      get_executor().add_executor(
        executor_ptr, this->passed_scheduler().get_sched_info(), false);
    }
    else {
      executor_ptr->operator()(execute_all);
      delete executor_ptr;
    }

    detail::scoped_drain_call x((blocking || force_evaluation) && !b_is_nested);

    // just to avoid warning on unused x...
    x.foo();

    return result_manager_t::finalize_result(result);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Function operator signature that engages blocking semantic of the
  /// PARAGRAPH.  The function call does not return until termination detection
  /// has succeeded and the result value for this location is available.
  //////////////////////////////////////////////////////////////////////
  result_type operator()(void)
  {
    return this->operator()((int) 0, true);
  }
}; // class paragraph

} } // namespace paragraph_impl::stapl


#undef STAPL_PARAGRAPH_DECLARATION_HEADER
#undef STAPL_PARAGRAPH_VIEW_TYPE_LIST

#endif // STAPL_PARAGRAPH_PARAGRAPH_HPP
