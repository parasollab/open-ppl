/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_EDGE_VIEW_HPP
#define STAPL_PARAGRAPH_EDGE_VIEW_HPP

#include <stapl/paragraph/edge_container/views/edge_view_fwd.h>
#include <stapl/paragraph/edge_container/views/edge_accessor.hpp>
#include <stapl/utility/empty_class.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Type metafunction to extract optional filter parameter from
///  variadic type parameter pack if specified, otherwise, use
///  @ref df_identity.  Also reflect the value_type the edge view refers
///  to, after applying the filter to the edge value.
//////////////////////////////////////////////////////////////////////
template<typename T, typename ...OptionalFilter>
struct compute_edge_view_filter;

template<typename T>
struct compute_edge_view_filter<T>
{
  typedef empty_class                        filter_base_type;

  typedef detail::df_identity<
     typename df_stored_type<T>::type>       filter_type;

  typedef T                                  value_type;

  static
  detail::df_identity<typename df_stored_type<T>::type>
  apply(empty_class const)
  { return detail::df_identity<typename df_stored_type<T>::type>(); }
};


template<typename T, typename Filter>
struct compute_edge_view_filter<T, Filter>
{
  typedef Filter                                     filter_base_type;
  typedef Filter                                     filter_type;
  typedef typename boost::result_of<Filter(T)>::type value_type;

  static
  Filter
  apply(Filter const& f)
  { return f; }
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Provides a view over the @p edge_container allowing PARAGRAPH tasks
///  to create data flow value consumption.
/// @ingroup pgEdgeViews
///
/// @tparam T The edge value type (i.e., the return type of the consumed task).
/// @tparam OptionalFilter Type of a functor applied to the the value produced
/// by a task of the edge_container prior data flow to the consumer.
///
/// The underlying edge container is non templated and is agnostic to the
/// edge value type.  The @p edge_view informs the @p edge_container of the
/// type via explicit template parameter specification when invoking methods.
///
/// @todo mutable keyword is required on @p m_reference_param until @p add_task
///   receives views by rvalue ref, allowing it non-const access to member
///   method (i.e., setup_flow()).  Remove mutable when this is fixed.
//////////////////////////////////////////////////////////////////////
template<typename T, typename... OptionalFilter>
class edge_view
  : private detail::compute_edge_view_filter<
      T, OptionalFilter...>::filter_base_type
    // inheritance for empty base optimization.
{
private:
  using base_type =
    typename detail::compute_edge_view_filter<
      T, OptionalFilter...>::filter_base_type;

public:
  using value_type =
    typename detail::compute_edge_view_filter<
     T, OptionalFilter...>::value_type;

  using reference  = lazy_edge_reference<value_type>;
  using index_type = std::size_t;

private:
  using stored_value_t   = typename df_stored_type<T>::type;
  using local_notifier_t = detail::edge_local_notifier_base;

  /// @brief Temporarily holds return value from call to
  /// @ref edge_container::setup_flow before forwarding to reference
  /// constructor in @ref operator[].
  mutable typename reference::constructor_param_type          m_reference_param;

  // Ensure this class is not unexpectedly heap allocated.
  void *operator new( size_t );
  void operator delete( void* );
  void *operator new[]( size_t );
  void operator delete[]( void* );

public:
  template<typename... OptionalFilterParam>
  explicit
  edge_view(OptionalFilterParam&&... filter)
    : base_type(std::forward<OptionalFilterParam>(filter)...)
  {
    static_assert(sizeof...(OptionalFilter) == sizeof...(OptionalFilterParam),
      "Wrong number of edge_view constructor parameters");
  }

  edge_view(edge_view const&)            = default;
  edge_view(edge_view&&)                 = default;
  edge_view& operator=(edge_view const&) = delete;

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Called by successor tasks to create an edge in
  /// the @ref edge_container.
  ///
  /// @param producer_tid The task identifier of the task to set up the
  ///   data flow from.
  /// @param notifier_ptr A pointer to the notifier that should be invoked
  ///   when the requested value is available on this location for consumption.
  ///
  /// Wraps the notifier appropriately based on whether persistency is
  /// enabled or not and then redirects to
  /// @ref edge_container::setup_flow.
  //////////////////////////////////////////////////////////////////////
  void setup_flow(index_type producer_tid,
                  local_notifier_t* notifier_ptr,
                  edge_container& container) const
  {
    using ephemeral_ptr_t  = local_notifier_t*;
    using persistent_ptr_t = boost::intrusive_ptr<local_notifier_t>;
    using notifier_t       =
      boost::function<void (executor_base&, value_type const&)>;

    m_reference_param = container.setup_flow<T>(
      producer_tid,
      container.is_persistent() ?
        notifier_t(bind(&local_notifier_t::operator(),
                        persistent_ptr_t(notifier_ptr), _1))
        : notifier_t(bind(&local_notifier_t::operator(),
                          ephemeral_ptr_t(notifier_ptr),  _1)),
      detail::compute_edge_view_filter<T, OptionalFilter...>::apply(*this));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a reference to the value produced by task with identifier
  /// @p producer_tid.
  ///
  /// The reference is backed by an entry in value cache of the underlying
  /// @ref edge_container.
  //////////////////////////////////////////////////////////////////////
  reference operator[](index_type producer_tid) const
  {
    return reference(std::move(m_reference_param));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Called to create a serialized edge reference if necessary
  ///  during paragraph task placement.
  //////////////////////////////////////////////////////////////////////
  reference transporter_reference(index_type producer_tid) const
  {
    stapl_assert(sizeof...(OptionalFilter) == 0,
                 "transporter protocol only supports full edge consumption");

    return reference(producer_tid);
  }

  using task_placement_dontcare = std::true_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Return locality information about the element specified by
  ///   the gid.
  /// @return A locality_qualifier, affinity specifier, as well as this
  ///   object's handle and associated location for this affinity.
  //////////////////////////////////////////////////////////////////////
  constexpr locality_info locality(index_type) const noexcept
  {
    return LQ_DONTCARE;
  }
}; // class edge_view


//////////////////////////////////////////////////////////////////////
/// @brief Provides a view over the input ports of an inter-paragraph dataflow
/// backed view argument, so that point to point dataflow to tasks within
/// a consuming paragraph can be initiated.
/// @ingroup pgEdgeViews
///
/// @tparam T The edge value type (i.e., the return type of the consumed task).
/// @tparam Optional Filter Type of a functor applied to the the value produced
/// by a task of the edge_container prior data flow to the consumer.
///
/// The underlying edge container is non templated and is agnostic to the
/// edge value type.  The @p edge_view informs the @p edge_container of the
/// type via explicit template parameter specification when invoking methods.
///
/// @todo mutable keyword is required on @p m_reference_param until @p add_task
///   receives views by rvalue ref, allowing it non-const access to member
///   method (i.e., setup_flow()).  Remove mutable when this is fixed.
//////////////////////////////////////////////////////////////////////
template<typename T, typename... OptionalFilter>
class pg_edge_view
  : private detail::compute_edge_view_filter<
      T, OptionalFilter...>::filter_base_type
    // inheritance for empty base optimization.
{
private:
  using base_type =
    typename detail::compute_edge_view_filter<
      T, OptionalFilter...>::filter_base_type;

public:
  using value_type =
    typename detail::compute_edge_view_filter<
     T, OptionalFilter...>::value_type;

  using reference  = pg_lazy_edge_reference<value_type>;
  using index_type = std::size_t;

private:
  using stored_value_t   = typename df_stored_type<T>::type;
  using local_notifier_t = detail::edge_local_notifier_base;

  /// @brief Temporarily holds return value from call to
  /// @ref edge_container::setup_flow before forwarding to reference
  /// constructor in @ref operator[].
  mutable typename reference::constructor_param_type          m_reference_param;

  /// @brief Reference to the local edge_container backing the
  /// inter-paragraph dataflow.
  edge_container&                                             m_ct;

  // Ensure this class is not unexpectedly heap allocated.
  void *operator new( size_t );
  void operator delete( void* );
  void *operator new[]( size_t );
  void operator delete[]( void* );

public:

  template<typename... OptionalFilterParam>
  pg_edge_view(edge_container& ct, OptionalFilterParam&&... filter)
    : base_type(std::forward<OptionalFilterParam>(filter)...),
      m_ct(ct)
  { }

  pg_edge_view(pg_edge_view const&)            = default;
  pg_edge_view(pg_edge_view&&)                 = default;
  pg_edge_view& operator=(pg_edge_view const&) = delete;

  void define_type(typer& t)
  {
    abort("Packing inter-paragraph edge_view should not occur");
    t.base<base_type>(*this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Called by successor tasks to create an edge in
  /// the @ref edge_container.
  ///
  /// @param producer_tid The task identifier of the task to set up the
  ///   data flow from.
  /// @param notifier_ptr A pointer to the notifier that should be invoked
  ///   when the requested value is available on this location for consumption.
  ///
  /// Wraps the notifier appropriately based on whether persistency is
  /// enabled or not and then redirects to
  /// @ref edge_container::setup_flow.
  //////////////////////////////////////////////////////////////////////
  void setup_flow(index_type producer_tid,
                  local_notifier_t* notifier_ptr,
                  edge_container&) const
  {
    using ephemeral_ptr_t  = local_notifier_t*;
    using persistent_ptr_t = boost::intrusive_ptr<local_notifier_t>;
    using notifier_t       =
      boost::function<void (executor_base&, value_type const&)>;

    m_reference_param = m_ct.setup_flow<T>(
      producer_tid,
      m_ct.is_persistent() ?
        notifier_t(bind(&local_notifier_t::operator(),
                        persistent_ptr_t(notifier_ptr), _1))
        : notifier_t(bind(&local_notifier_t::operator(),
                          ephemeral_ptr_t(notifier_ptr),  _1)),
      detail::compute_edge_view_filter<T, OptionalFilter...>::apply(*this));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a reference to the value produced by task with identifier
  /// @p producer_tid.
  ///
  /// The reference is backed by an entry in value cache of the underlying
  /// @ref edge_container.
  //////////////////////////////////////////////////////////////////////
  reference operator[](index_type producer_tid) const
  {
    return reference(std::move(m_reference_param), m_ct);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Called to create a serialized edge reference if necessary
  ///  during paragraph task placement.
  //////////////////////////////////////////////////////////////////////
  reference transporter_reference(index_type producer_tid) const
  {
    abort("Transporting inter-paragraph edge_view should not occur");

    return reference(producer_tid);
  }

  using task_placement_dontcare = std::true_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Return locality information about the element specified by
  ///   the gid.
  /// @return A locality_qualifier, affinity specifier, as well as this
  ///   object's handle and associated location for this affinity.
  //////////////////////////////////////////////////////////////////////
  constexpr locality_info locality(index_type) const noexcept
  {
    return LQ_DONTCARE;
  }
}; // class pg_edge_view

} // namespace stapl

#endif // ifndef STAPL_PARAGRAPH_EDGE_VIEW_HPP
