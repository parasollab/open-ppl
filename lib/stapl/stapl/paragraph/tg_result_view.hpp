/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_TG_RESULT_VIEW_HPP
#define STAPL_PARAGRAPH_TG_RESULT_VIEW_HPP

#include <unordered_map>
#include <type_traits>

#include <boost/function.hpp>
#include <boost/intrusive/list.hpp>
#include <boost/utility/result_of.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Notifier used in @ref result_container for a consumer
/// @ref paragraph at the same level of nesting as the producer.
/// Holds various filters and mappers used to guide the dataflow process.
//////////////////////////////////////////////////////////////////////
template<typename Index, typename T, typename PortFilter,
         typename ValueFilter, typename LocationMapper>
class result_ct_sibling_notifier
{
private:
  std::size_t           m_consumer;
  rmi_handle::reference m_consumer_handle;
  location_type         m_parent_location;
  PortFilter            m_port_filter;
  ValueFilter           m_value_filter;
  LocationMapper        m_location_mapper;

public:
  result_ct_sibling_notifier(std::size_t consumer,
                             rmi_handle::reference consumer_handle,
                             location_type parent_location,
                             PortFilter port_filter,
                             ValueFilter value_filter,
                             LocationMapper location_mapper)
    : m_consumer(consumer),
      m_consumer_handle(std::move(consumer_handle)),
      m_parent_location(parent_location),
      m_port_filter(std::move(port_filter)),
      m_value_filter(std::move(value_filter)),
      m_location_mapper(std::move(location_mapper))
  { }

  void operator()(Index index, T const& val,
                  task_graph& tg, location_type producer_location) const
  {
    using flowed_value_t = typename boost::result_of<ValueFilter(T)>::type;

    // Check if consumer wants this specific index of my output port.
    if (!m_port_filter(index))
      return;

    std::function<void (task_graph&)> f;

    const location_type sibling_consumer_loc = m_location_mapper(index).first;

    // If the target location in the consumer paragraph is the same as my
    // location then I can used m_parent_location to directly dataflow to the
    // proper affinity instead of routing through the parent's nested pg
    // directory.
    if (tg.get_location_id() == sibling_consumer_loc)
    {
      using mem_fun_t =
        void (task_graph::*)(location_type,
                             rmi_handle::reference, location_type,
                             Index, flowed_value_t const&);

      constexpr mem_fun_t mem_fun =
        &task_graph::child_nested_sibling_set_element_send<flowed_value_t>;


      f = std::bind(mem_fun, std::placeholders::_1,
                    m_parent_location, m_consumer_handle,
                    m_location_mapper(index).first,
                    index, m_value_filter(val));
    }
    else
    {
      using mem_fun_t =
        void (task_graph::*)(std::size_t, rmi_handle::reference, location_type,
                             Index, flowed_value_t const&);

      constexpr mem_fun_t mem_fun =
        &task_graph::child_route_dataflow<flowed_value_t>;

      f = std::bind(mem_fun, std::placeholders::_1,
                    m_consumer, m_consumer_handle,
                    m_location_mapper(index).first,
                    index, m_value_filter(val));
    }

    // Get a direct pointer to the parent pg on this affinity if it has already
    // been created.
    auto parent_pg_ptr = resolve_handle<task_graph>(tg.parent_handle());

    if (parent_pg_ptr != nullptr)
    {
      f(*parent_pg_ptr);

      return;
    }

    // else
    //
    // The parent PARAGRAPH doesn't exist at this affinity yet.  Add a message
    // that will be checked for when it is constructed.
    add_pg_message(tg.parent_handle(), std::move(f));
  }
}; // class result_ct_sibling_notifier


//////////////////////////////////////////////////////////////////////
/// @brief Notifier used in @ref result_container for a consumer
/// @ref paragraph who is the parent of producer (i.e., output port
/// to output port dataflow). Holds various filters and mappers used
/// to guide the dataflow process.
//////////////////////////////////////////////////////////////////////
template<typename Index, typename T>
class result_ct_parent_notifier
{
private:
  using mapper_t      = boost::function<size_t (size_t)>;
  using port_filter_t = boost::function<bool   (size_t)>;

  mapper_t         m_mapper;
  port_filter_t    m_port_filter;

public:
  result_ct_parent_notifier(mapper_t mapper, port_filter_t port_filter)
    : m_mapper(std::move(mapper)),
      m_port_filter(std::move(port_filter))
  { }

  void operator()(Index index, T const& val,
                  task_graph& tg, location_type) const
  {
    auto& parent_pg = *resolve_handle<task_graph>(tg.parent_handle());

    auto& parent_result_ct =
      down_cast<detail::result_container<T>&>(parent_pg.get_result_container());

    if (m_port_filter(index))
      parent_result_ct.receive_value(m_mapper(index), val);

  }
}; // class result_ct_parent_notifier


template<typename T>
result_container<T>::
result_container(bool b_one_sided, bool b_spanned, task_graph& tg)
  : m_plug_ref_cnt(1),
    m_tg_ref_cnt(0),
    m_succ_cnt(b_spanned ? tg.num_successors() : 0),
    m_b_orphaned(false),
    m_b_one_sided(b_one_sided),
    m_request_flow_call_count(0),
    m_tg_ptr(&tg)
{
  // We reserve not only because we know the size at this point, but there
  // is a bug (maybe in STAPL, but likely in gcc 4.8 / boost) causing the
  // location mapper to be prematurely destroyed for some consumers
  // during the growth function of the vector.
  m_consumers.reserve(m_succ_cnt);

  stapl_assert(!tg_initializer::is_initializing(),
    "result_container constructor found tg_initializer::is_initializing()");
}


template<typename T>
void
result_container<T>::
add_successor(successor_notifier_t&& notifier)
{
  stapl_assert(has_outstanding_consumers(),
               "Invalid successor count specification");

  ++m_request_flow_call_count;

  // Add myself to the list of known consumers.  Any values flowed from this
  // producer paragraph will be directly sent to me in unordered manner with
  // regard to the flush of buffer values about to be sent.
  m_consumers.push_back(notifier);

  auto iter    = m_values.begin();
  auto e_iter  = m_values.end();

  for ( ; iter != e_iter; ++iter)
    notifier(
      iter->index(), iter->value(), *m_tg_ptr, m_tg_ptr->get_location_id());

  // If all consumers are now known and I'm the only request_flow() call on
  // the call stack, free buffered values.
  if (!has_outstanding_consumers() && m_request_flow_call_count == 1)
  {
    if (m_b_one_sided)
      this->erase_values();

    m_tg_ptr->pulse_terminator();
  }

  --m_request_flow_call_count;

  try_orphaned_delete();
}


template<typename T>
template<typename PortFilter, typename ValueFilter, typename LocationMapper>
void
result_container<T>::
request_flow(std::size_t consumer,
             rmi_handle::reference consumer_handle,
             location_type parent_location,
             PortFilter port_filter,
             ValueFilter value_filter,
             LocationMapper location_mapper)
{
  this->add_successor(
    result_ct_sibling_notifier<
      index_type, stored_value_t, PortFilter, ValueFilter, LocationMapper>
      (consumer, consumer_handle, parent_location,
       port_filter, value_filter, location_mapper));
}


template<typename T>
void
result_container<T>::
request_parent_flow(boost::function<size_t (size_t)> mapper,
                    boost::function<bool   (size_t)> port_filter)
{
  add_successor(
    result_ct_parent_notifier<
      index_type, stored_value_t>(std::move(mapper), std::move(port_filter)));
}


template<typename T>
void
result_container<T>::receive_value(index_type index, stored_value_t const& val)
{
  // Range based for loop purposefully not used, as size may increase during
  // execution of loop, due to incoming request_flow calls during armi
  // polling.  Keep iterating until any new consumers are notified as well.
  for (size_t idx = 0; idx < m_consumers.size(); ++idx)
    m_consumers[idx](index, val, *m_tg_ptr, m_tg_ptr->get_location_id());

  // If all consumers are already known, they were notified above and
  // there is no need to buffer.
  if (m_b_one_sided && !has_outstanding_consumers())
    return;

  m_values.push_front(*new entry_t(index, val));
}


//////////////////////////////////////////////////////////////////////
/// @brief Accessor used by @ref result_view that allows notification
///   requests and operations requests to wait until deferred value
///   is available.
/// @ingroup pgResultView
//////////////////////////////////////////////////////////////////////
template<typename View>
struct tg_result_accessor
  : public accessor_base<typename View::value_type, tg_result_accessor<View>>
{
private:
  template <typename Derived, typename A, typename C, typename D>
  friend class iterator_facade;

  friend class ::stapl::accessor_core_access;

public:
  using index_type     = typename View::index_type;

private:
  using value_type     = typename View::value_type;
  using stored_value_t = typename df_stored_type<value_type>::type;

  /// @brief The view providing access to the element this accessor refers to.
  View               m_view;

  /// @brief The index of the reference element in @p m_view.
  index_type         m_idx;

  //////////////////////////////////////////////////////////////////////
  /// @brief Required for proxies and views to perform dynamic localization
  /// in the PARAGRAPH.
  //////////////////////////////////////////////////////////////////////
  bool is_local() const
  {
    return true;
  }

public:
  index_type index() const
  {
    return m_idx;
  }

  tg_result_accessor(null_reference const&)
    : m_idx()
  { }

  tg_result_accessor(View const& view, index_type idx)
    : m_view(view), m_idx(idx)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return true if the version of the value this accessor refers to
  /// is available for use on this location.
  //////////////////////////////////////////////////////////////////////
  bool available(void) const
  { return this->m_view.available(this->m_idx); }


  //////////////////////////////////////////////////////////////////////
  /// @brief Register a callback notifier to be invoked when the underlying
  /// value is available.  Notifier invoked immediately if the value is already
  /// available.
  ///
  /// Storage and servicing of the notifier is handled by the container
  /// underlying the view associated with this accessor.
  //////////////////////////////////////////////////////////////////////
  template<typename Notifier>
  void request_notify(Notifier const& notifier) const
  { this->m_view.container().request_notify(this->m_idx, notifier); }


  /// @brief Type tag this accessor as referring to a value that might not yet
  /// be available.  Enables behavior such lazy evaluation of unary and binary
  /// operations on associated proxies.
  using deferred_evaluation_accessor_ = void;

  //////////////////////////////////////////////////////////////////////
  /// @brief Return true if this edge accessor is logically null and does not
  /// provide access to any actual value.
  //////////////////////////////////////////////////////////////////////
  bool is_null() const
  { return m_view.is_null(); }


  template<typename F>
  typename boost::result_of<F(value_type)>::type
  apply_get(F const& f) const
  {
    while (!this->available())
    {
      get_executor()();
      rmi_poll();
    }
    return f(m_view.apply_get(m_idx, f));
  }

  void define_type(typer& t)
  {
    stapl_assert(0, "result_accessors should not be packed.");

    t.member(m_view);
    t.member(m_idx);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Models array_view concept over the result_container of a PARAGRAPH.
///   The result container is held by pointer whose lifetime is managed among
///   instances of this class via reference counting.
/// @ingroup pgResultView
///
/// @todo @p apply_get method may be dead, superseded by @p get_reference.
/// Remove if this is the case.
///
/// @todo control access to assignment operator. Use with core_access to idiom
/// for internal stapl calls and hide from public use.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct result_view
{
  friend struct tg_result_accessor<result_view>;

  using container_t    = result_container<T>;

private:
  using stored_value_t = typename df_stored_type<T>::type;
  using accessor_t     = tg_result_accessor<result_view>;

  /// @brief Pointer to result_container with intrusive reference counting
  /// that this class updates on copy construction, assignment, and destruction.
  container_t*                                        m_container_sptr;

  /// @brief Can view be plugged into predecessor PARAGRAPH input view ports.
  bool                                                m_b_is_plug;

  /// @brief Is instance inside another PARAGRAPH as one of it's input views.
  bool                                                m_b_transitioned;

public:
  using value_type = T;
  using reference  = proxy<value_type, accessor_t>;
  using index_type = std::size_t;
  // old views use cid_type
  using cid_type   = index_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Default constructed result_view is considered null.
  //////////////////////////////////////////////////////////////////////
  result_view()
    : m_container_sptr(nullptr),
      m_b_is_plug(false),
      m_b_transitioned(false)
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor called in @ref result_manager when PARAGRAPH is
  /// initializing return value storage
  //////////////////////////////////////////////////////////////////////
  result_view(bool b_one_sided, bool b_spanned, task_graph& tg)
    : m_container_sptr(new container_t(b_one_sided, b_spanned, tg)),
      m_b_is_plug(true),
      m_b_transitioned(false)
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Copy construct updates reference counts and also detects if it is
  /// being called during the initialization of a predecessor PARAGRAPH.  If
  /// this is the case:
  /// (1) set the m_b_transitioned flag so we know we have a new consumer.
  /// (2) unset m_b_plug, copies within this PARAGRAPH (when creating its
  ///     tasks don't register as additional consumers.
  ///
  //////////////////////////////////////////////////////////////////////
  result_view(result_view const& other)
    : m_container_sptr(other.m_container_sptr),
      m_b_is_plug(other.m_b_is_plug && !tg_initializer::is_initializing()),
      m_b_transitioned(other.m_b_is_plug && tg_initializer::is_initializing())
  {
    if (!this->is_null())
      this->container().increment_ref_cnt(m_b_is_plug, m_b_transitioned);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Destroy object after decrementing container reference count.
  //////////////////////////////////////////////////////////////////////
  ~result_view()
  {
    if (this->is_null())
      return;

    const std::size_t ref_cnt =
      this->container().decrement_ref_cnt(m_b_is_plug);

    stapl_assert(ref_cnt != std::numeric_limits<std::size_t>::max(),
                 "~result_view get a negative df_ct ref count");

    // could warn if a plug is deleting (means we missed a chance at reuse
    if (ref_cnt == 0)
      m_container_sptr->try_delete();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Assignment operator that adjusts the reference count of associated
  /// result_container.
  //////////////////////////////////////////////////////////////////////
  result_view& operator=(result_view const& rhs)
  {
   if (tg_initializer::is_initializing())
   {
     std::cout << "whoa\n";
   }

    stapl_assert(!tg_initializer::is_initializing(),
      "result_view::operator=() found tg_initializing");

    if (&rhs == this)
      return *this;

    if (!this->is_null())
      this->container().decrement_ref_cnt(m_b_is_plug);

    m_container_sptr = rhs.m_container_sptr;
    m_b_is_plug      = rhs.m_b_is_plug;
    m_b_transitioned = false;

    if (!this->is_null())
      this->container().increment_ref_cnt(m_b_is_plug, m_b_transitioned);

    return *this;
  }

  void define_type(typer&)
  {
    stapl_assert(0, "result_views aren't packable");
  }

  container_t& container() const
  {
    return *m_container_sptr;
  }

  size_t version(void) const
  {
    return 0;
  }

  size_t validate(void) const
  {
    return true;
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Return a reference (i.e., proxy) to an element of this view.
  /// @param idx Index (i.e., location) of result requested.
  //////////////////////////////////////////////////////////////////////
  reference operator[](index_type idx) const
  {
    return reference(accessor_t(*this, idx));
  }

  using deferred_evaluation_view_ = void;

  //////////////////////////////////////////////////////////////////////
  /// @brief Return true iff the specified index in the container has been
  /// written to from the corresponding internal PARAGRAPH task.
  //////////////////////////////////////////////////////////////////////
  bool available(index_type idx) const
  {
    return this->container().available(idx);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Allows successor PARAGRAPHs to register a callback, effectively
  /// setting up cross PARAGRAPH dependence edges.
  ///
  /// @param idx       Index (i.e., location) of requested result.
  /// @param notifier  Function operator to call when value is available.
  //////////////////////////////////////////////////////////////////////
  template<typename Notifier>
  void request_notify(index_type idx, Notifier notifier) const
  {
    this->container().request_notify(idx, notifier);
  }

private:
  //
  // Methods use by tg_result_accessor
  //
  //////////////////////////////////////////////////////////////////////
  /// @brief Check for logically null view (i.e., default constructed).
  //////////////////////////////////////////////////////////////////////
  bool is_null(void) const
  {
    return m_container_sptr == nullptr;
  }

public:
  /////////////////////////////////////////////////////////////////////
  /// @brief Return reference to specified element in result container.
  //////////////////////////////////////////////////////////////////////
  stored_value_t const& get_reference(index_type idx) const
  {
    return this->container().get_reference(idx);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return copy of specified element in result container.
  //////////////////////////////////////////////////////////////////////
  value_type get_element(index_type idx) const
  {
    return this->container().get_element(idx);
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Apply function object to an element in container and
  /// return the result.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  typename Functor::result_type
  apply_get(index_type idx, Functor func) const
  {
    return this->container().apply_get(idx, func);
  }
}; // struct result_view


//////////////////////////////////////////////////////////////////////
/// @brief Event handler which receives signals from inter-paragraph
///   data flow and termination detection and waits until both are
///   received before calling a notifier which informs the parent
///   paragraph that this one-sided initialized child paragraph has
///   completed and provides the corresponding dataflow.
///
/// Guards against possible race condition where data flow is triggered
/// prior to the success of termination detection, allowing the parent
/// pg to be notified, succeed in its termination detection, and be
/// deleted prior to the child.  The @ref gang_executor does not allow
/// child deletion after parents (i.e., tear down must be bottom up).
//////////////////////////////////////////////////////////////////////
template<typename T>
class one_sided_wait_td_notifier
{
private:
  using notifier_type = boost::function<void (T const&)>;

  bool                                     m_b_value_set;
  bool                                     m_b_td_succeeded;
  T                                        m_value;
  notifier_type                            m_notifier;

  void fire_value(void)
  {
    m_notifier(std::move(m_value));
    delete this;
  }

public:
  one_sided_wait_td_notifier(notifier_type notifier)
   : m_b_value_set(false),
     m_b_td_succeeded(false),
     m_notifier(std::move(notifier))
  { }

  void set_value(T const& value, bool b_stealable)
  {
    if (b_stealable)
      m_value = std::move(const_cast<T&>(value));
    else
      m_value = value;

    m_b_value_set = true;

    if (m_b_td_succeeded)
      this->fire_value();
  }

  void notify_td(void)
  {
    m_b_td_succeeded = true;

    stapl_assert(m_b_value_set, "data flow not available and TD succeeded");

    this->fire_value();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of class for paragraphs with void return type.
///   In this case, there is no inter paragraph dataflow to hold.
///   Notification is hooked directly into termination detection.
//////////////////////////////////////////////////////////////////////
template<>
class one_sided_wait_td_notifier<void>
{
private:
  using notifier_type = boost::function<void (void)>;

  notifier_type                            m_notifier;

public:
  one_sided_wait_td_notifier(notifier_type notifier)
    : m_notifier(std::move(notifier))
  { }

  void notify_td(void)
  {
    m_notifier();
    delete this;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Encapsulates access to edge entry and version storage objects
/// for @ref paragraph, to be used for out edge paragraph data flow
/// optimizations (e.g., zero copy). Empty class if result type is void.
/// @tparam T Result type of the paragraph.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct edge_info_holder
{
private:
  using entry_t = detail::edge_entry_base;

  using version_storage_t =
    detail::edge_version_storage<
     typename df_stored_type<
       typename boost::result_of<
         detail::df_identity<T>(
           typename df_stored_type<T>::type)
       >::type
     >::type>;

  entry_t*                         m_entry_ptr;
  version_storage_t*               m_version_storage_ptr;

public:
  edge_info_holder(void)
    : m_entry_ptr(nullptr),
      m_version_storage_ptr(nullptr)
  { }

  void set(tuple<entry_t*, version_storage_t*> const& edge_info)
  {
    stapl_assert(
      m_entry_ptr == nullptr &&m_version_storage_ptr == nullptr,
      "found already initialized state");

    m_entry_ptr           = get<0>(edge_info);
    m_version_storage_ptr = get<1>(edge_info);
  }

  entry_t& entry(void)
  {
    stapl_assert(m_entry_ptr != nullptr, "uninitialized entry");

    return *m_entry_ptr;
  }

  version_storage_t& version_storage(void)
  {
    stapl_assert(m_version_storage_ptr != nullptr,
      "uninitialized version storage");

    return *m_version_storage_ptr;
  }
}; // struct edge_info_holder


template<>
struct edge_info_holder<void>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief This class is used internally by the PARAGRAPH to manage the setup of
///   storage for the PARAGRAPH's result as well as setting up the notification
///   flow from the internal task on each location in the PARAGRAPH gang.
/// @ingroup pgResultView
///
/// @sa paragraph::operator()
///
/// @todo The partial specializations of this class that follow this primary
/// template can be removed (and should be with a little code restructuring).
/// The composition::map_view logic should be moved to composition::map and
/// void return value can probably be removed as well.  When this occurs,
/// The code from this class can just be placed in paragraph::operator(), the
/// only call site.
//////////////////////////////////////////////////////////////////////
template<typename Return, bool b_has_span = false>
struct result_manager
{
  using edge_type       = Return;

  using result_type     = typename result_view<edge_type>::reference;

  using tmp_result_type = result_type;

  using stored_value_t    = typename df_stored_type<edge_type>::type;

  using notifier_t        =
    boost::function<void (std::size_t, stored_value_t const&)>;

  using os_td_wait_t      = one_sided_wait_td_notifier<edge_type>;

  /// @brief The notifier this PARAGRAPH invokes when the task marked as the
  /// result task for this location finishes execution.
  notifier_t                        m_result_notifier;

  /// @brief Holds information about internal edge that is serving outgoing
  /// paragraph edge so that zero copy /move can be attempted upon exit.
  edge_info_holder<edge_type>       m_out_edge_info;

  result_container_base* get_result_container(void)
  {
    abort("Invalid invocation of get_result_container");
    return nullptr;
  }

  void reset(void)
  { }

  bool has_outstanding_consumers(void) const
  { return false; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Setup the result_view and underlying result_container storage
  /// for the PARAGRAPH, and register the appropriate callback with PARAGRAPH
  //////////////////////////////////////////////////////////////////////
  result_type apply(task_graph& tg, bool one_sided)
  {
    using res_vw_t = result_view<edge_type>;
    using res_ct_t = typename res_vw_t::container_t;

    res_vw_t result(one_sided, false, tg);

    if (!one_sided)
    {
      res_ct_t* ct_ptr = &result.container();

      m_result_notifier =
        [ct_ptr](std::size_t idx, stored_value_t const& val)
          { ct_ptr->receive_value(idx, val); };
    }

    return result[tg.get_location_id()];
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Signature for non-void return types. Redirect to the
  /// @ref edge_container
  ///
  /// @todo Implement zero copy data flow for collectively spawned paragraphs.
  /// Current support is limited to one-sided paragraphs.
  //////////////////////////////////////////////////////////////////////
  void set_result_impl(std::size_t index,
                       task_graph::task_id_t task_id,
                       task_graph& tg)
  {
    tg.count_task();
    tg.count_local_task();

    m_out_edge_info.set(
      tg.edges().template set_result<edge_type>(
        task_id,
        boost::function<void (executor_base&, edge_type const&)>(
          [&tg, this](executor_base&, edge_type const& val)
          {
            this->m_result_notifier(tg.get_location_id(), val);
            tg.result_processed();
          }
    )));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief If the return type of the paragraph is non-void, initializes
  /// an instance of @ref one_sided_wait_td_notifier to tie parent paragraph
  /// notification to both this paragraph's dataflow and termination
  /// detection.
  ///
  /// @todo Consider rearchitecting to allow unconditional move out of edge
  /// entry. It's possible that the result notifier is called before other
  /// consumers of the producer task have completed.  Right now, as we wait for
  /// TD to outflow, there's no negative to deferring steal until TD succeeds.
  /// However, if we relaxed this (i.e., consumption of side affects and
  /// outflow are separately requestable at finer granularity), there is
  /// potential to increase critical path.
  //////////////////////////////////////////////////////////////////////
  os_td_wait_t* set_os_result_notifier(task_graph& tg, result_type&)
  {
    location_type child_location        = tg.get_location_id();
    location_type parent_location       = tg.parent_location();
    rmi_handle::reference parent_handle = tg.parent_handle();
    bool b_has_succs                    = tg.has_successors();
    size_t task_id                      = tg.task_id();

    auto ptr = new os_td_wait_t(
      [child_location, parent_location, parent_handle,
       b_has_succs, task_id](Return const& val)
      {
        if (child_location == 0)
        {
          p_object* p = resolve_handle<p_object>(parent_handle);
          stapl_assert(p != nullptr, "Failed parent pg handle resolution");
          gang g(*p);

          using mem_fun_t =
            void (task_graph::*)(std::size_t, bool, Return&& val);

          const mem_fun_t mem_fun = &task_graph::processed_remote;

          async_rmi(parent_location, parent_handle, mem_fun,
                    task_id, b_has_succs,
                    std::move(const_cast<Return&>(val)));
        }
      });

    m_result_notifier =
      [ptr, this](std::size_t, stored_value_t const& val)
      {
        const bool b_stealable =
          m_out_edge_info.entry().out_edge_stealable()
          && m_out_edge_info.version_storage().is_direct_storage();

        if (b_stealable)
          ptr->set_value(m_out_edge_info.version_storage().steal(), true);
        else
          ptr->set_value(val, false);
      };

    return ptr;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief For the primary class template, this method is an identity op.
  ///
  /// @param res The PARAGRAPH result from this location.
  //////////////////////////////////////////////////////////////////////
  result_type& finalize_result(result_type& res)
  {
    return res;
  }
}; // struct result_manager<Return, false> (primary template)


template<typename Return>
struct result_manager<Return, true>
{
public:
  using edge_type       = Return;
  using result_type     = result_view<edge_type>;
  using tmp_result_type = result_type;
  using stored_value_t  = typename df_stored_type<edge_type>::type;
  using os_td_wait_t    = one_sided_wait_td_notifier<void>;

  /// @brief The notifier this PARAGRAPH invokes when the task marked as the
  /// result task for this location finishes execution.
  result_type m_result;

  result_container_base* get_result_container(void)
  {  return &m_result.container(); }

  bool has_outstanding_consumers(void) const
  {
    return m_result.container().has_outstanding_consumers();
  }

  void reset(void)
  {
    return m_result.container().reset();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Setup the result_view and underlying result_container storage
  /// for the PARAGRAPH, and register the appropriate callback with PARAGRAPH
  ///
  /// @note Current implementation for persistent paragraphs invalidates
  /// return values held explicitly by user code via proxy backed by the
  /// @ref tg_result_accessor, when a subsequent execution is initiated.
  /// Could instantiate new result container as in non-span case, but
  /// that would require transfer of registered consumer state.
  //////////////////////////////////////////////////////////////////////
  result_type apply(task_graph& tg, bool one_sided)
  {
    using res_vw_t = result_view<edge_type>;
    using res_ct_t = typename res_vw_t::container_t;

    if (tg.called_before())
      return m_result;

    res_vw_t result(one_sided, true, tg);

    if (!one_sided)
      stapl_assert(0, "Collective Portbased PARAGRAPHs not implemented yet.");

    m_result = result;

    return result;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Signature for non-void return types. Redirect to the
  /// @ref edge_container
  ///
  /// @todo Implement zero copy data flow for collectively spawned paragraphs.
  /// Current support is limited to one-sided paragraphs.
  //////////////////////////////////////////////////////////////////////
  void set_result_impl(std::size_t index,
                       task_graph::task_id_t task_id,
                       task_graph& tg)
  {
    tg.count_task();
    tg.count_local_task();

    auto& ct = m_result.container();

    using notifier_t =
      boost::function<void (executor_base&, stored_value_t const&)>;

    notifier_t f =
      [&tg, index, &ct](executor_base&, stored_value_t const& val)
      {
        ct.receive_value(index, val);
        tg.result_processed();
      };

    tg.edges().template set_result<edge_type>(task_id, f);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief If the return type of the paragraph is non-void and we are
  /// spanned, value consumer will receive data point to point. Furthermore,
  /// firing of the values will be accounted for in global TD for signal
  /// consumers.  Hence, the one sided notification is based solely on
  /// global TD.
  //////////////////////////////////////////////////////////////////////
  os_td_wait_t* set_os_result_notifier(task_graph& tg, result_type& vw)
  {
    // Make copies of these, as when this notifier is called, the paragraph
    // object storing them will have been destroyed.
    location_type child_location        = tg.get_location_id();
    rmi_handle::reference parent_handle = tg.parent_handle();
    bool b_notify_all_locations         = tg.parent_notify_all_locations();

    return new os_td_wait_t(
      [child_location, parent_handle, b_notify_all_locations](void)
      {
        if (child_location != 0 && !b_notify_all_locations)
          return;

        task_graph* p = resolve_handle<task_graph>(parent_handle);

        stapl_assert(p != nullptr, "Failed parent pg handle resolution");
        gang g(*p);

        // For now, if scheduler has task_placement_all_local type switch
        // flipped, assume that add_expected_tasks() interface has also been
        // used, and inform all locations (not just 0) that the nested
        // PARAGRAPH has terminated.
        const location_type my_loc = p->get_location_id();

        if (b_notify_all_locations || child_location == 0)
        {
          // Need to use an async here instead of pointer returned from
          // resolve handle to make sure that processed to the parent doesn't
          // happen until this call returns.
          async_rmi(my_loc, parent_handle,
                    &task_graph::processed_remote_void, 0, 0);
        }
      }
    );
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief For the primary class template, this method is an identity op.
  ///
  /// @param res The PARAGRAPH result from this location.
  //////////////////////////////////////////////////////////////////////
  result_type& finalize_result(result_type& res)
  {
    return res;
  }
}; // struct result_manager<Return, true>


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for PARAGRAPH's with a void return value.
/// @ingroup pgResultView
///
/// @todo For symmetry with the other result_manager cases, we should return
/// some kind of proxy to void that can be queried to check that the task graph
/// has completed execution, analogous to explicit predecessor to @p add_task.
//////////////////////////////////////////////////////////////////////
template<>
struct result_manager<void, false>
{
  using tmp_result_type = size_t;
  using result_type     = void;
  using edge_type       = void;
  using os_td_wait_t    = one_sided_wait_td_notifier<void>;

  result_container_base* get_result_container(void)
  {
    abort("Invalid invocation of get_result_container");
    return nullptr;
  }

  bool has_outstanding_consumers(void) const
  { return false; }

  void reset(void)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Effectively a noop to maintain uniform interface.  No result
  /// container, etc need be created no task value is propagated on the
  /// PARAGRAPH's outgoing edge.
  //////////////////////////////////////////////////////////////////////
  size_t apply(task_graph const&, bool)
  {
    return 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a nulltpr (i.e., noop) when the return type of the
  ///   paragraph is void, as there's no need to create an instance of
  ///   @ref one_sided_wait_td_notifier.
  //////////////////////////////////////////////////////////////////////
  os_td_wait_t* set_os_result_notifier(task_graph& tg, tmp_result_type)
  {
    location_type child_location        = tg.get_location_id();
    location_type parent_location       = tg.parent_location();
    rmi_handle::reference parent_handle = tg.parent_handle();
    bool b_has_succs                    = tg.has_successors();

    return new os_td_wait_t(
      [child_location, parent_location, parent_handle, b_has_succs](void)
      {
        if (child_location == 0)
        {
          p_object* p = resolve_handle<p_object>(parent_handle);
          stapl_assert(p != nullptr, "Failed parent pg handle resolution");
          gang g(*p);

          async_rmi(parent_location, parent_handle,
                    &task_graph::processed_remote_void, 0, b_has_succs);
        }
      }
    );
  }

  void set_result_impl(std::size_t, task_graph::task_id_t, task_graph&)
  { abort("Set result called on void return paragraph"); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Changes return value that leaves the PARAGRAPH from size_t that
  /// the result task produces to the external void return type of PARAGRAPH.
  //////////////////////////////////////////////////////////////////////
  void finalize_result(size_t)
  { }
}; // struct result_manager<void, false>


template<>
struct result_manager<void, true>
  : public result_manager<void, false>
{
  os_td_wait_t* set_os_result_notifier(task_graph& tg, tmp_result_type)
  {
    // Make copies of these, as when this notifier is called, the paragraph
    // object storing them will have been destroyed.
    location_type child_location        = tg.get_location_id();
    rmi_handle::reference parent_handle = tg.parent_handle();
    bool b_notify_all_locations         = tg.parent_notify_all_locations();

    return new os_td_wait_t(
      [child_location, parent_handle, b_notify_all_locations](void)
      {
        if (child_location != 0 && !b_notify_all_locations)
          return;

        task_graph* p = resolve_handle<task_graph>(parent_handle);

        stapl_assert(p != nullptr, "Failed parent pg handle resolution");
        gang g(*p);

        // For now, if scheduler has task_placement_all_local type switch
        // flipped, assume that add_expected_tasks() interface has also been
        // used, and inform all locations (not just 0) that the nested
        // PARAGRAPH has terminated.
        const location_type my_loc = p->get_location_id();

        if (b_notify_all_locations || child_location == 0)
        {
          // Need to use an async here instead of pointer returned from
          // resolve handle to make sure that processed to the parent doesn't
          // happen until this call returns.
          async_rmi(my_loc, parent_handle,
                    &task_graph::processed_remote_void, 0, 0);
        }
      }
    );

  }
};

} // namespace detail

template<typename View>
struct accessor_traits<detail::tg_result_accessor<View>>
{
  using is_localized = std::true_type;
};

} // namespace stapl

#endif // STAPL_PARAGRAPH_TG_RESULT_VIEW_HPP
