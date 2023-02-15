/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_EDGE_ACCESSOR_HPP
#define STAPL_PARAGRAPH_EDGE_ACCESSOR_HPP

#include <stapl/paragraph/paragraph_fwd.h>
#include <stapl/paragraph/edge_container/utility.hpp>
#include <stapl/paragraph/edge_container/edge_container.h>

#include "edge_accessor_fwd.hpp"

#include <type_traits>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of stand-alone begin for rvalue references
/// to a proxy with an edge_accessor.
///
/// Example:
/// auto it = stapl::begin(proxy<T, edge_accessor<T>>(t));
///
/// @ingroup iteratorSpecializations
//////////////////////////////////////////////////////////////////////
template <typename T>
auto begin(stapl::proxy<T, edge_accessor<T>>&& t) ->
  decltype(
    const_cast<stapl::proxy<T, edge_accessor<T>> const&&>(t).begin())
{
  return const_cast<stapl::proxy<T, edge_accessor<T>> const&&>(t).begin();
}

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of stand-alone begin for lvalue references
/// to a proxy with an edge_accessor.
///
/// Example:
/// proxy<T, edge_accessor<T>> p(t);
/// auto it = stapl::begin(p);
///
/// @ingroup iteratorSpecializations
//////////////////////////////////////////////////////////////////////
template <typename T>
auto begin(stapl::proxy<T, edge_accessor<T>>& t) ->
  decltype(
    const_cast<stapl::proxy<T, edge_accessor<T>> const &>(t).begin())
{
  return const_cast<stapl::proxy<T, edge_accessor<T>> const &>(t).begin();
}

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of stand-alone end for rvalue references
/// to a proxy with an edge_accessor.
///
/// Example:
/// auto it = stapl::end(proxy<T, edge_accessor<T>>(t));
///
/// @ingroup iteratorSpecializations
//////////////////////////////////////////////////////////////////////
template <typename T>
auto end(stapl::proxy<T, edge_accessor<T>>&& t) ->
  decltype(
    const_cast<stapl::proxy<T, edge_accessor<T>> const&&>(t).end())
{
  return const_cast<stapl::proxy<T, edge_accessor<T>> const&&>(t).end();
}

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of stand-alone end for lvalue references
/// to a proxy with an edge_accessor.
///
/// Example:
/// proxy<T, edge_accessor<T>> p(t);
/// auto it = stapl::end(p);
///
/// @ingroup iteratorSpecializations
//////////////////////////////////////////////////////////////////////
template <typename T>
auto end(stapl::proxy<T, edge_accessor<T>>& t) ->
  decltype(
    const_cast<stapl::proxy<T, edge_accessor<T>> const &>(t).end())
{
  return const_cast<stapl::proxy<T, edge_accessor<T>> const &>(t).end();
}


namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction that extracts a reference to the user level value
/// that was stored in the @ref edge_container.
/// @ingroup pgEdgeViews
///
/// Used to unwrap proxies held in @ref proxy_holder objects.
/////////////////////////////////////////////////////////////////////
template<typename T>
struct extract_value_ref_mf
{
  static T const& apply(T const& t)
  { return t; }
};


//////////////////////////////////////////////////////////////////////
/// @brief specialization for edge views without a filtering function
/// whose edge type is a proxy.
///
/// Extract the proxy held by the @ref proxy_holder that the
/// @ref edge_container wrapped it in.
//////////////////////////////////////////////////////////////////////
template<typename T, typename A>
struct extract_value_ref_mf<proxy<T,A>>
{
  template<typename Q>
  static proxy<T, A> const& apply(Q const& q)
  { return q.m_proxy; }
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Underlying access mechanism for proxies of values stored in
///   the @ref paragraph @ref edge_container.
/// @ingroup pgEdgeViews
//////////////////////////////////////////////////////////////////////
template<typename T>
struct edge_accessor
{
private:
  friend class accessor_core_access;

  using value_type = T;

  value_type const& m_val_ref;

public:
  edge_accessor(value_type const& val)
    : m_val_ref(val)
  { }

  value_type read(void) const
  {
    return m_val_ref;
  }

  template<typename Class, typename Rtn, typename... Args>
  Rtn const_invoke(Rtn (Class::* const pmf)(Args...) const,
                   typename std::decay<Args>::type const&... args) const

  {
    return (m_val_ref.*pmf)(args...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke function object @p f, passing the accessor's referenced
  /// value as a parameter.
  ///
  /// Required interface of accessor framework (@ref accessor_base) to
  /// generate @p invoke signatures used by proxy implementation).
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  typename std::result_of<F(value_type)>::type
  apply_get(F const& f) const
  {
    return f(m_val_ref);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Added to remove the compilation error for calling add task
  /// on views over proxies of aggregate types (e.g., proxy<vector>)
  ///
  /// @todo should be removed if the call chain to define_type in
  /// edge_accessor is removed
  //////////////////////////////////////////////////////////////////////
  void define_type(typer&)
  {
    abort("edge_accessor's define type should not be called.");
  }
}; // struct edge_accessor


template<typename T>
struct accessor_traits<edge_accessor<T>>
{
  using is_localized = std::true_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Reference object by @ref edge_view when creating a task.
/// Delays the creation of @ref edge_accessor until the address of the
/// flowed value is available (i.e., is it directly in edge_entry managed
/// storage or available via cross-location, immutable sharing.
///
/// @todo Use @ref trivial_accessor for small, basic T types, and replace
/// edge_accessor usage with ref_accessor.
/// @ingroup pgEdgeViews
//////////////////////////////////////////////////////////////////////
template<typename T>
struct lazy_edge_reference
{
  typedef tuple<
    detail::edge_entry_base*,
    detail::edge_version_storage<
      typename df_stored_type<T>::type>*>              constructor_param_type;

  static detail::edge_entry_base                       serialized;

  typedef T                                            value_type;
  typedef typename df_stored_type<value_type>::type    stored_value_t;
  typedef detail::edge_version_storage<stored_value_t> storage_t;
  typedef size_t                                       index_type;
  typedef edge_accessor<T>                             accessor_type;
  typedef proxy<value_type, accessor_type>             reference_type;

  /// @brief The associated edge entry with the value.  Used to update
  ///   reference counting and attempt entry eviction when possible.
  ///
  /// There are three possible states:
  ///  nullptr - accessor has moved into another object and is now invalid.
  ///  &serialized - accessor has been serialized for out of gang
  ///    transmission.
  ///  other - accessor is standard, entry_backed variety.
  detail::edge_entry_base*                             m_entry_ptr;

  union {
    /// @brief A pointer directly to the storage of the referenced element in
    /// the container of @p m_view.
    storage_t*                                         m_storage_ptr;

    /// @brief The task identifier for this accessor.  Set if serialized.
    size_t                                             m_index;
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Primary constructor receiving pointers to @ref edge_entry
  /// for metadata (i.e., ref counting) updates and @ref edge_version_storage
  /// for value access.
  //////////////////////////////////////////////////////////////////////
  lazy_edge_reference(constructor_param_type const& param)
    : m_entry_ptr(get<0>(param)),
      m_storage_ptr(get<1>(param))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor used when initializing for the
  ///   serialization of an out-of-gang paragraph task.
  //////////////////////////////////////////////////////////////////////
  lazy_edge_reference(size_t index)
    : m_entry_ptr(&serialized),
      m_index(index)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Copy construct which increments the reference count to element
  /// in the underlying @ref edge_entry.
  //////////////////////////////////////////////////////////////////////
  lazy_edge_reference(lazy_edge_reference const& other)
    : m_entry_ptr(other.m_entry_ptr),
      m_storage_ptr(other.m_storage_ptr)
  {
    m_entry_ptr->increment_local_consumers();
  }

  lazy_edge_reference(lazy_edge_reference&& other)
    : m_entry_ptr(other.m_entry_ptr),
      m_storage_ptr(other.m_storage_ptr)
  {
    other.m_entry_ptr = nullptr;
  }

  lazy_edge_reference& operator=(lazy_edge_reference const&) = delete;

  //////////////////////////////////////////////////////////////////////
  /// @brief Destructor is a noop, verify that a prior call to release()
  /// has reset the entry pointer member to nullptr.
  //////////////////////////////////////////////////////////////////////
  ~lazy_edge_reference(void)
  {
    stapl_assert(m_entry_ptr == nullptr, "found live entry_ptr");
  }

  index_type index(void) const
  {
    stapl_assert(m_entry_ptr != nullptr, "index() found null entry_ptr");
    return m_entry_ptr == &serialized ? m_index : m_entry_ptr->tid();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Create real reference to the flowed value as requested by
  /// the task.
  //////////////////////////////////////////////////////////////////////
  reference_type get_reference(void) const
  {
    stapl_assert(m_entry_ptr != nullptr && m_entry_ptr != &serialized,
      "get_reference called on uninitialized lazy_edge_reference");

    return reference_type(accessor_type(
      detail::extract_value_ref_mf<T>::apply(m_storage_ptr->value())));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Checks whether the object underlying this edge value can be
  /// stolen by the task and either (a) moved into the workfunction or
  /// (b) moved to the outgoing edge (for identity tasks).  Guards calls
  /// to @ref steal.
  //////////////////////////////////////////////////////////////////////
  bool stealable(void) const
  {
    return m_entry_ptr->stealable();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an rvalue reference to the underlying object for the
  /// edge.  Ownership can then be transferred into the associated task.
  //////////////////////////////////////////////////////////////////////
  stored_value_t&& steal(void) const
  {
    return m_storage_ptr->steal();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Get reference to underlying edge value object.
  //////////////////////////////////////////////////////////////////////
  stored_value_t const& get_storage_ref(void) const
  {
    return m_storage_ptr->value();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns true if version storage directly holds the value and
  /// false is held in an @ref immutable_shared wrapper.
  //////////////////////////////////////////////////////////////////////
  bool is_direct_storage(void) const
  {
    return m_storage_ptr->is_direct_storage();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return copy of shared wrapper so that it can be forwarded
  /// to outgoing task edge in identity tasks.
  //////////////////////////////////////////////////////////////////////
  immutable_shared<stored_value_t> get_shared_wrapper(void) const
  {
    return m_storage_ptr->wrapper();
  }

  void define_type(typer& t)
  {
    stapl_assert(m_entry_ptr == &serialized,
      "edge_accessor::define_type found non serialized entry");

    t.member(m_index);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Required for proxies and views to perform dynamic localization
  /// in the PARAGRAPH.
  ///
  /// The edge_accessor always refers to data available locally, as data
  /// flow is initialized to whatever location the associated task runs on.
  //////////////////////////////////////////////////////////////////////
  bool is_local(void) const
  {
    return true;
  }

  void pre_execute()
  { }

  void post_execute()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Notifies underlying @ref edge_entry that this reference is
  ///   Signature for ephemeral paragraphs, eviction is attempted via
  ///   @ref tg_callback if associated with an in-gang task.  Otherwise, the
  ///   associated out-of-gang task owns the entry and deletes it directly.
  //////////////////////////////////////////////////////////////////////
  template<typename TGCallback>
  void release(TGCallback const& cb)
  {
    stapl_assert(m_entry_ptr != nullptr && m_entry_ptr != &serialized,
                 "release called on invalid lazy_reference");

    m_entry_ptr->decrement_local_consumers();

    if (!cb.valid())
      cb.tg().edges().try_base_eviction(*m_entry_ptr);
    else
    {
      stapl_assert(m_entry_ptr->evictable(),
        "transported lazy_referenec not evictable");

      stapl_assert(!m_entry_ptr->is_linked(),
        "edge entry is linked when attempting deletion");

      delete m_entry_ptr;
    }

    m_entry_ptr = nullptr;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Notifies underlying @ref edge_entry that this reference is
  ///   done using the value.  Signature for persistent paragraphs.
  //////////////////////////////////////////////////////////////////////
  void release(void)
  {
    stapl_assert(m_entry_ptr != nullptr && m_entry_ptr != &serialized,
                 "release called on invalid lazy_reference");

    stapl_assert(!m_entry_ptr->is_basic_edge_entry(),
                 "Persistent, out of gang entry detected, not supported");

    m_entry_ptr->decrement_local_consumers();
    m_entry_ptr = nullptr;
  }
}; // struct lazy_edge_reference


//////////////////////////////////////////////////////////////////////
/// @brief Refinement of @ref lazy_edge_reference concept for inter-paragraph
/// port based dataflow.  Holds pointer to associated edge_container to attempt
/// evictions, instead of using @ref task_graph reference provided
/// by callers of release().
//////////////////////////////////////////////////////////////////////
template<typename T>
struct pg_lazy_edge_reference
  : public lazy_edge_reference<T>
{
  edge_container* m_ct_ptr;

  using base_t = lazy_edge_reference<T>;

  //////////////////////////////////////////////////////////////////////
  /// @brief Primary constructor receiving pointers to @ref edge_entry
  /// for metadata (i.e., ref counting) updates and @ref edge_version_storage
  /// for value access.
  //////////////////////////////////////////////////////////////////////
  pg_lazy_edge_reference(typename base_t::constructor_param_type const& param,
                         edge_container& ct)
    : base_t(param), m_ct_ptr(&ct)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor used when initializing for the
  ///   serialization of an out-of-gang paragraph task.
  //////////////////////////////////////////////////////////////////////
  pg_lazy_edge_reference(size_t index)
    : base_t(index), m_ct_ptr(nullptr)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Override of base class method which attempts eviction of
  /// entry in @ref edge_container that is held as a member.
  //////////////////////////////////////////////////////////////////////
  template<typename TGCallback>
  void release(TGCallback const& cb)
  {
    stapl_assert(this->m_entry_ptr != nullptr
                 && this->m_entry_ptr != &base_t::serialized,
                 "release called on invalid lazy_reference");

    this->m_entry_ptr->decrement_local_consumers();

    if (!cb.valid())
      m_ct_ptr->try_base_eviction(*this->m_entry_ptr);
    else
    {
      stapl_assert(this->m_entry_ptr->evictable(),
        "transported lazy_reference not evictable");

      stapl_assert(!this->m_entry_ptr->is_linked(),
        "edge entry is linked when attempting deletion");

      delete this->m_entry_ptr;
    }

    this->m_entry_ptr = nullptr;
  }

  void release(void)
  { base_t::release(); }
}; // struct pg_lazy_edge_reference


template<typename T>
struct accessor_traits<lazy_edge_reference<T>>
{ using is_localized = std::true_type; };


template<typename T>
struct accessor_traits<pg_lazy_edge_reference<T>>
{ using is_localized = std::true_type; };


template<typename T>
detail::edge_entry_base
lazy_edge_reference<T>::serialized(0, false);

} // namespace stapl

#endif // ifndef STAPL_PARAGRAPH_EDGE_ACCESSOR_HPP

