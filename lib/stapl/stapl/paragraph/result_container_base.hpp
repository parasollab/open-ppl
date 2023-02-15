/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_RESULT_CONTAINER_BASE_HPP
#define STAPL_PARAGRAPH_RESULT_CONTAINER_BASE_HPP

#include <stapl/paragraph/edge_container/utility.hpp>
#include <stapl/paragraph/edge_container/notifier_list.hpp>

namespace stapl {

namespace paragraph_impl {

class task_graph;

} // namespace paragraph_impl

namespace detail {

using paragraph_impl::task_graph;

//////////////////////////////////////////////////////////////////////
/// @brief Base class used to allow @ref paragraph class hierarchy to
/// provide access to @ref result_container where the result type is erased.
//////////////////////////////////////////////////////////////////////
struct result_container_base
{
  //////////////////////////////////////////////////////////////////////
  /// @todo Exists solely to allow @ref down_cast to succeed by making the
  /// class polymorphic.  Refactor to remove need for this.
  //////////////////////////////////////////////////////////////////////
  virtual ~result_container_base() = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Called by parent PARAGRAPH to request flow from this output
  /// port to some set of output ports on the parent.
  ///
  /// @param mapper Applied to the index of the output port to map to a
  /// possibly different index in the parent.
  ///
  /// @param port_filter Functor called on each output value to determine
  /// whether or not it should be flowed to the consumer.
  //////////////////////////////////////////////////////////////////////
  virtual void request_parent_flow(
    boost::function<size_t (size_t)> mapper,
    boost::function<bool (size_t)>   port_filter) = 0;
}; // struct result_container_base


//////////////////////////////////////////////////////////////////////
/// @brief Encapsulate notifier list behavior used by @p result_container.
///   Namely two interfaces: one to add a new notifier to a list and another
///   to invoke all notifiers, delete them, and empty the list.
/// @ingroup pgResultView
///
/// @tparam Notifier Type of notifier objects held by the list.
//////////////////////////////////////////////////////////////////////
template<typename Notifier>
struct intrusive_notifier_list
{
private:
  using hook_t          = boost::intrusive::list_base_hook<>;
  using entry_t         = notifier_entry<Notifier, hook_t>;
  using notifier_list_t = boost::intrusive::list<entry_t>;

  notifier_list_t                            m_notifications;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Add a notifier to the list.
  //////////////////////////////////////////////////////////////////////
  void push_back(Notifier const& f)
  {
    m_notifications.push_back(*new entry_t(f));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Iterate over list, removing notifiers, invoking their function
  /// operators, and then deleting the notifier objects.
  //////////////////////////////////////////////////////////////////////
  void flush(void)
  {
    while (m_notifications.size())
    {
      entry_t& entry_ref = m_notifications.front();

      m_notifications.pop_front();

      entry_ref();

      delete &entry_ref;
    }
  }
}; // struct intrusive_notifier_list


//////////////////////////////////////////////////////////////////////
/// @brief Storage class for buffering value in the @ref result_container
/// when there are still unspecified consumers.
//////////////////////////////////////////////////////////////////////
template<typename Index, typename T, typename Hook>
class result_value_storage_entry
  : public Hook
{
private:
  Index   m_index;
  T       m_value;

public:
  result_value_storage_entry(Index const& index, T const& value)
    : m_index(index), m_value(value)
  { }

  STAPL_USE_MANAGED_ALLOC(result_value_storage_entry)

  Index const& index(void) const
  {
    return m_index;
  }

  T const& value(void) const
  {
    return m_value;
  }
}; // result_value_storage_entry


//////////////////////////////////////////////////////////////////////
/// @brief This object is allocated on each location of a PARAGRAPH as storage
///   for that location's return value of the PARAGRAPH.  The key feature is
///   that its lifetime can extend beyond the PARAGRAPH itself and is driven by
///   the return value's use by other computation.
/// @ingroup pgResultView
///
/// @tparam T The return type of the PARAGRAPH.
///
/// @todo resolve operator[] and get_reference return_type mismatch.
///
/// @todo In the current implementation, the object is constructed in the
/// PARAGRAPH's (possibly nested) group, and deleted outside by ref counting
/// by the parent's group.  This can cause errors with per location storage
/// (different pools for each virtual location).  Use of STAPL_USE_MANAGED_ALLOC
/// is disabled for now until this is fixed either by moving allocation outside
/// the PARAGRAPH (either before PARAGRAPH or with corresponding move after
/// PARAGRAPH) or another approach
///
/// @todo use Boost.Optional / similar technique to avoid default construction
/// and subsequent assignment operator call of m_value
/// Copy construct or move in value when the @p receive_value is called.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct result_container
  : public result_container_base
{
public:
  using index_type     = std::size_t;
  using value_type     = T;

private:
  using stored_value_t = typename df_stored_type<T>::type;

  using entry_t        =
    result_value_storage_entry<
      index_type, stored_value_t,
      boost::intrusive::make_slist_base_hook<>::type
    >;

  using list_t         =
    typename boost::intrusive::make_slist<
      entry_t, boost::intrusive::constant_time_size<false>
    >::type;

  /// @brief Number of result_view objects existing outside of other PARAGRAPH
  /// that may potentially be plugged into other PARAGRAPHs, creating new
  /// successors.
  size_t                                              m_plug_ref_cnt;

  /// @brief Number of result_view objects existing inside other PARAGRAPH.
  size_t                                              m_tg_ref_cnt;

  /// @brief Number of successor PARAGRAPHs.
  size_t                                              m_succ_cnt;

  using successor_notifier_t =
    boost::function<
      void (index_type, stored_value_t const&, task_graph&, location_type)>;

  /// @brief Notifiers to consuming paragraphs (parent or sibling).
  std::vector<successor_notifier_t>                   m_consumers;

  /// @brief Storage for local PARAGRAPH return value
  list_t                                              m_values;

  /// @brief True if deletion attempted during a notification flush.
  bool                                                m_b_orphaned;

  /// @brief True if this paragraph was created nested, one-sided.
  bool                                                m_b_one_sided;

  /// @brief Used to detect reentrant calls to request_flow(), ensuring
  /// @p m_values is flushed by lowest call on the stack, so that all
  /// flow request receive the cached values.
  std::size_t                                         m_request_flow_call_count;

  task_graph*                                         m_tg_ptr;

  ////////////////////////////////////////////////////////////////////////
  /// @brief Remove all cached values during cleanup or reset for persistent
  /// call.
  ////////////////////////////////////////////////////////////////////////
  void erase_values(void)
  {
    m_values.erase_and_dispose(
      m_values.begin(), m_values.end(),
      [](entry_t* entry_ptr) { delete entry_ptr; }
    );
  }

  ////////////////////////////////////////////////////////////////////////
  /// @brief Called at end of flushing operations (e.g., request_flow()) to
  /// see if (a) this object has been flagged for deletion via a call to
  /// try_delete() further up the call stack and (b) this is the last
  /// remaining call into the object on the stack.  If both conditions hold,
  /// delete the object.
  //////////////////////////////////////////////////////////////////////
  void try_orphaned_delete(void)
  {
    if (m_b_orphaned && m_request_flow_call_count == 0)
      delete this;
  }

  ////////////////////////////////////////////////////////////////////////
  /// @brief Adds a successor notifier to the list of known consumers and
  /// streams already buffered values through the notifier, if the consumer
  /// has arrived after some ports have already fired.
  ////////////////////////////////////////////////////////////////////////
  void add_successor(successor_notifier_t&& notifier);

public:
  ////////////////////////////////////////////////////////////////////////
  /// @brief Plug count initialized to one, representing the view object
  /// that constructed this object.
  //////////////////////////////////////////////////////////////////////
  explicit result_container(bool b_one_sided, bool b_spanned, task_graph& tg);


  // see todo on class documentation
  // STAPL_USE_MANAGED_ALLOC(result_container)

  //////////////////////////////////////////////////////////////////////
  /// @todo Consider a better assert to make sure all values have been
  /// flowed prior to destruction.
  //////////////////////////////////////////////////////////////////////
  ~result_container()
  {
    stapl_assert(m_b_one_sided || !m_values.empty(),
      "~result_container: value not set");

    this->erase_values();
  }


  void reset(void)
  {
    this->erase_values();
  }


  bool has_outstanding_consumers(void) const
  {
    return m_consumers.size() < m_succ_cnt;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Called by PARAGRAPHs at same level of nesting as this one to
  /// request dataflow from this output port to one of their input ports.
  /// This method is called on each location of the producer by the matching
  /// locations on the consumer.
  ///
  /// @param consumer The task id of the consumer paragraph used in its parent.
  ///
  /// @param consumer_handle rmi handle reference to the input port on the
  /// destination PARAGRAPH.
  ///
  /// @param parent_location The location of the parent paragraph colocated
  /// with the matching location on the consumer paragraph (flow will be
  /// routed through there).
  ///
  /// @param port_filter Used to select which pins of the producer port
  ///   are forwarded to this consumer.
  ///
  /// @param value_filter Applied to each flowed pin's value, prior to
  ///   dataflow to the consumer.
  ///
  /// @param location_mapper Maps pin_ids to the location in the consumer
  ///        @ref paragraph they are flowed to.
  ///
  /// @todo Extend logic to allow for deletion of buffer when traversing
  /// for data flow in cases when possible.
  //////////////////////////////////////////////////////////////////////
  template<typename PortFilter, typename ValueFilter, typename LocationMapper>
  void request_flow(std::size_t consumer,
                    rmi_handle::reference consumer_handle,
                    location_type parent_location,
                    PortFilter port_filter,
                    ValueFilter value_filter,
                    LocationMapper location_mapper);


  //////////////////////////////////////////////////////////////////////
  /// @copydoc result_container_base::request_parent_flow
  //////////////////////////////////////////////////////////////////////
  void request_parent_flow(boost::function<size_t (size_t)> mapper,
                           boost::function<bool   (size_t)> port_filter) final;


  //////////////////////////////////////////////////////////////////////
  /// @brief Check if a deletion has been placed on hold while
  /// notifications are being flushed. If it has, flag the object as orphaned
  /// so that the flushing function will handle deletion when it is finished.
  /// If no hold has been placed, delete the object now.
  ///
  /// Called by result_view when combined reference count is zero, which
  ///happens as part of the associated PARAGRAPH's deletion.
  //////////////////////////////////////////////////////////////////////
  void try_delete()
  {
    stapl_assert(!has_outstanding_consumers(),
      "try delete found outstanding consumers");

    if (m_request_flow_call_count == 0)
    {
      delete this;
      return;
    }

    // else
    m_b_orphaned = true;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Target of callback from internal PARAGRAPH task that specifies the
  /// result for this location (i.e., task_id was passed to @p set_result).
  ///
  /// Copy task's value into m_value and flush notifications to successor
  /// PARAGRAPHS.
  ///
  /// @todo Perform move on @p val if legal, into a single async and if
  /// not there, into the buffer.
  //////////////////////////////////////////////////////////////////////
  void receive_value(index_type index, stored_value_t const& val);


  //////////////////////////////////////////////////////////////////////
  /// @brief Check if local PARAGRAPH result is available.
  //////////////////////////////////////////////////////////////////////
  bool available(index_type idx) const
  {
    return std::find_if(m_values.begin(), m_values.end(),
                        [&](entry_t const& entry)
                          { return entry.index() == idx; }
                        ) != m_values.end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Checks to see if global flow has been detected...
  //////////////////////////////////////////////////////////////////////
  bool available(void) const
  {
    abort("available() not implemented for port based paragraphs");
    return false;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return copy of local PARAGRAPH result
  //////////////////////////////////////////////////////////////////////
  value_type get_element(index_type idx) const
  {
    return this->get_reference(idx);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return reference to local PARAGRAPH result
  //////////////////////////////////////////////////////////////////////
  stored_value_t const& get_reference(index_type idx) const
  {
    auto iter =
      std::find_if(m_values.begin(), m_values.end(),
                   [&](entry_t const& entry) { return entry.index() == idx; });

    stapl_assert(iter != m_values.end(), "out value not set");

    return iter->value();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return reference to local PARAGRAPH result
  //////////////////////////////////////////////////////////////////////
  value_type const& operator[](index_type idx) const
  {
    return this->get_reference(idx);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return result of functor application to local PARAGRAPH result.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  typename Functor::result_type
  apply_get(index_type idx, Functor func) const
  {
    return func(this->operator[](idx));
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Update reference counts based on new result_view object
  /// instantiation.
  ///
  /// @param b_is_plug      Is calling result_view a plug?
  /// @param b_transitioned Has calling result_view transitioned to be a
  ///                       consumer?
  //////////////////////////////////////////////////////////////////////
  void increment_ref_cnt(bool b_is_plug, bool b_transitioned)
  {
    if (b_is_plug)
      ++m_plug_ref_cnt;
    else
      ++m_tg_ref_cnt;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Decrement the appropriate reference counter and return the
  /// aggregate number of references to this container based on @p b_is_plug.
  /// The caller view will delete this object iff the return value is 0.
  //////////////////////////////////////////////////////////////////////
  std::size_t decrement_ref_cnt(bool b_is_plug)
  {
    const std::size_t rtn = b_is_plug ?
      --m_plug_ref_cnt + m_tg_ref_cnt
      : m_plug_ref_cnt + --m_tg_ref_cnt;

    return rtn;
  }

  size_t version(void) const
  {
    return 0;
  }
}; // struct result_container

} // namespace detail

} // namespace stapl

#endif // STAPL_PARAGRAPH_RESULT_CONTAINER_BASE_HPP
