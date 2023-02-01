/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_AGGREGATED_EDGE_VIEW_HPP
#define STAPL_PARAGRAPH_AGGREGATED_EDGE_VIEW_HPP

#include <stapl/paragraph/edge_container/views/edge_accessor.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief View that provides access to an aggregated collection of
///   task data flow values from the edge_container.
/// @ingroup pgEdgeViews
///
/// @tparam ProxyContainer Type of container used by class to store references
/// to each edge value.
///
/// This class is the reference type of @ref aggregated_edge_view and is the
/// view type passed to user workfunctions when the associated and task uses the
/// aggregated version of @ref consume function of the public PARAGRAPH
/// interface.
///
/// @todo We can leverage the fact that we know that the instance of this
/// class stored in @ref Task viewset member will be the last called and
/// invoke release() on the elements of the array.  Instead of a shared_ptr
/// a union with an ProxyContainer instance and a pointer to it (along
/// with interpretation bit) can avoid additional heap alloc and counter
/// ops associated with the shared pointer.
//////////////////////////////////////////////////////////////////////
template<typename ProxyContainer>
class aggregated_edge_subview
{
private:
  /// @brief Holds references to all edge values that are part of the
  /// aggregated edge view defined during successor task creation.
  std::shared_ptr<ProxyContainer> m_container_sptr;

public:
  using reference  = typename ProxyContainer::value_type::reference_type;
  using index_type = std::size_t;

  aggregated_edge_subview(ProxyContainer ct)
    : m_container_sptr(std::make_shared<ProxyContainer>(std::move(ct)))
  { }

  aggregated_edge_subview(aggregated_edge_subview const& other) = default;
  aggregated_edge_subview(aggregated_edge_subview&& other)      = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Return one of the edge value proxies stored in this view
  /// class
  ///
  /// @param idx Index of edge value to access.  Not indexed by task
  ///   identifier.  Range is [0..size()-1].
  //////////////////////////////////////////////////////////////////////
  reference operator[](std::size_t idx) const
  {
    stapl_assert(idx < m_container_sptr->size(),
      "aggregated_edge_subview::operator[]: out of bounds");

    return m_container_sptr->operator[](idx).get_reference();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Required callback to views invoked before a task using the view
  /// has executed.
  ///
  /// For this class, there's nothing to do.
  //////////////////////////////////////////////////////////////////////
  void pre_execute(void) const noexcept
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Required callback to views invoked after a task using the view
  /// has executed.
  ///
  /// For this class, there's nothing to do.
  //////////////////////////////////////////////////////////////////////
  void post_execute(void) const noexcept
  { }

  std::size_t size() const
  {
    return m_container_sptr->size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Required for proxies and views to perform dynamic localization in
  /// the PARAGRAPH.
  ///
  /// Values backed by the @p edge_container are always available locally, as
  /// data flow is initialized to whatever location the associated task runs on.
  //////////////////////////////////////////////////////////////////////
  constexpr bool is_local(void) const noexcept
  {
    return true;
  }

  void define_type(typer&)
  {
    abort("aggregated edge data shouldn't be serialized");
  }

  template<typename... Args>
  void release(Args&&... args)
  {
    for (auto&& elem : *m_container_sptr)
      elem.release(std::forward<Args>(args)...);
  }
}; // class aggregated_edge_subview


//////////////////////////////////////////////////////////////////////
/// @brief Provides access to a dynamically selected group of values in the
///   @p edge_container.  The index type of the view, instead of being a single
/// task identifier is logically a sequence of task identifiers.
/// @ingroup pgEdgeViews
///
/// @param T The edge value type of the consumed tasks (All consumed tasks
///   must have the same edge type).
/// @param Filter The type of the filter to apply to all edge values prior to
///   making them available to target workfunction.
///
/// @todo mutable keyword is required on @p m_reference_param until @p add_task
///   receives views by rvalue ref, allowing it non-const access to member
///   method (i.e., setup_flow()).  Remove mutable when this is fixed.
//////////////////////////////////////////////////////////////////////
template<typename T, typename... OptionalFilter>
class aggregated_edge_view
  : private detail::compute_edge_view_filter<
      T, OptionalFilter...>::filter_base_type
    // inheritance for empty base optimization.
{
public:
  using value_type =
    typename detail::compute_edge_view_filter<
      T, OptionalFilter...>::value_type;

private:
  using base_type =
    typename detail::compute_edge_view_filter<
      T, OptionalFilter...>::filter_base_type;

  using base_reference_t = lazy_edge_reference<value_type>;

  /// @brief Temporarily holds base_references before
  /// construction of @ref aggregated_edge_subview in @ref operator[].
  mutable std::vector<base_reference_t>          m_reference_param;

public:
  /// @brief The index type is currently a std::vector but could be anything
  /// that implements the array_view concept.
  using index_type = std::vector<std::size_t>;

  /// @brief Reference type is a view over all values defined by variables
  /// of type @p index-type.
  using reference = aggregated_edge_subview<std::vector<base_reference_t>>;

  template<typename... OptionalFilterParam>
  explicit
  aggregated_edge_view(OptionalFilterParam&&... filter)
    : base_type(std::forward<OptionalFilterParam>(filter)...)
  {
    static_assert(sizeof...(OptionalFilter) == sizeof...(OptionalFilterParam),
      "Wrong number of edge_view constructor parameters");
  }

  aggregated_edge_view(aggregated_edge_view const&)            = default;
  aggregated_edge_view(aggregated_edge_view&&)                 = default;
  aggregated_edge_view& operator=(aggregated_edge_view const&) = delete;

  void define_type(typer &t)
  {
    t.base<base_type>(*this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Called by successor tasks to create an edge in
  /// the @ref edge_container.
  ///
  /// @param producer_tids Task identifiers of the tasks to set up the
  ///   data flow from.
  /// @param notifier_ptr A pointer to the notifier that should be invoked
  ///   when the requested values are available on this location for
  ///   consumption.
  ///
  /// Wraps the notifier appropriately based on whether persistency is enabled
  /// or not and then redirects to @ref edge_container::setup_flow.
  /// One redirect per consumed task.
  //////////////////////////////////////////////////////////////////////
  void setup_flow(index_type const& producer_tids,
                  detail::edge_local_notifier_base* notifier_ptr,
                  edge_container& container) const
  {
    using local_notifier_t = detail::edge_local_notifier_base;
    using ephemeral_ptr_t  = local_notifier_t*;
    using persistent_ptr_t = boost::intrusive_ptr<local_notifier_t>;
    using notifier_t       =
      boost::function<void (executor_base&, value_type const&)>;

    m_reference_param.reserve(producer_tids.size());

    for (auto&& producer_tid : producer_tids)
    {
      m_reference_param.emplace_back(
        container.setup_flow<T>(
          producer_tid,
          container.is_persistent() ?
            notifier_t(bind(&local_notifier_t::operator(),
                            persistent_ptr_t(notifier_ptr), _1))
            : notifier_t(bind(&local_notifier_t::operator(),
                              ephemeral_ptr_t(notifier_ptr),  _1)),
          detail::compute_edge_view_filter<T, OptionalFilter...>::apply(*this))
      );
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a view over the values produced by tasks specified
  /// in identifier list @p producer_tids.
  ///
  /// @sa aggregated_edge_subview
  //////////////////////////////////////////////////////////////////////
  reference operator[](index_type const& producer_tids) const
  {
    return reference(std::move(m_reference_param));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Called to create a serialized edge reference if necessary
  ///  during paragraph task placement.
  //////////////////////////////////////////////////////////////////////
  reference transporter_reference(index_type consumed_tid) const
  {
    abort("transporter protocol only supports full edge consumption");

    return operator[](index_type());
  }

  using task_placement_dontcare = std::true_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Return locality information about the element specified by
  ///   the gid.
  /// @return A locality_qualifier, affinity specifier, as well as this
  ///   object's handle and associated location for this affinity.
  //////////////////////////////////////////////////////////////////////
  locality_info locality(index_type) const noexcept
  {
    return LQ_DONTCARE;
  }

  ~aggregated_edge_view()
  {
    stapl_assert(m_reference_param.size() == 0, "unexpected entries");
  }
}; // class aggregated_edge_view

} // namespace stapl

#endif // ifndef STAPL_PARAGRAPH_AGGREGATED_EDGE_VIEW_HPP
