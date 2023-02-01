/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_TASK_MIGRATION_HPP
#define STAPL_PARAGRAPH_TASK_MIGRATION_HPP

#include <stapl/views/proxy.h>

#include <stapl/paragraph/edge_container/views/edge_accessor.hpp>
#include <stapl/paragraph/edge_container/edge_container.h>
#include <stapl/paragraph/edge_container/utility.hpp>
#include <stapl/paragraph/edge_container/views/edge_view_fwd.h>

#include <stapl/utility/tuple.hpp>

namespace stapl {

namespace paragraph_impl {

template<typename T>
struct proxy_holder;


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for instances of @ref proxy backed by
///   the @ref edge_accessor.
/// @ingroup paragraph
///
/// Hold a copy of the value and the corresponding task identifier directly.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct proxy_holder<lazy_edge_reference<T>>
{
  /// @brief A copy of the value the proxy refers to.
  T           m_val;

  /// @brief The task identifier this of the task that produced the value.
  std::size_t m_tid;

  proxy_holder(lazy_edge_reference<T>&& ref, edge_container& ct)
    : m_val(ref.get_reference()),
      m_tid(ref.index())
  {
    ref.release(tg_callback(&ct.tg()));
  }

  proxy_holder& operator=(proxy_holder const& rhs) = delete;

  void define_type(typer& t)
  {
    t.member(m_val);
    t.member(m_tid);
  }
}; //class proxy_holder


//////////////////////////////////////////////////////////////////////
/// @brief Given an edge value type @p T, this metafunction returns the
///   type serialized during task migration.
/// @ingroup paragraph
//////////////////////////////////////////////////////////////////////
template<typename View>
struct migration_packer
{
  using type = View;

  template<typename ViewParam>
  static type apply(ViewParam&& view, edge_container&)
  { return std::forward<ViewParam>(view); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for instances of @ref proxy backed by the
///   @ref edge_accessor with default filtering (i.e., no filtering).
///   Wrap these proxies in @ref proxy_holder.
/// @ingroup paragraph
//////////////////////////////////////////////////////////////////////
template<typename T>
struct migration_packer<lazy_edge_reference<T>>
{
  using type = proxy_holder<lazy_edge_reference<T>>;

  static type apply(lazy_edge_reference<T>&& ref, edge_container& ct)
  { return proxy_holder<lazy_edge_reference<T>>(std::move(ref), ct); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Given an edge value type @p T, this metafunction returns the
///   unpacked type of the egde, post task migration.
/// @ingroup paragraph
//////////////////////////////////////////////////////////////////////
template<typename View>
struct migration_unpacker
{
  using type = View;

  //////////////////////////////////////////////////////////////////////
  /// @brief Function operator
  /// @param vw The deserialized view to process, post migration.
  /// @param edge_vw The edge_view (without filtering) for this location.
  //////////////////////////////////////////////////////////////////////
  static type apply(View const& vw, edge_container&)
  { return vw; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for instances of edge container proxies which
///   have been wrapped in a @ref proxy_holder.  Reconstructs the proxy
///   after inserting value in the local @ref edge_container.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct migration_unpacker<proxy_holder<lazy_edge_reference<T>>>
{
  using packed_type = proxy_holder<lazy_edge_reference<T>>;
  using type        = lazy_edge_reference<T>;

  static type apply(packed_type const& holder, edge_container& edge_ct)
  {
    return type(edge_ct.set_migrated_value<T>(holder.m_tid, holder.m_val));
  }
};

} // namespace paragraph_impl

} // namespace stapl

#endif // STAPL_PARAGRAPH_TASK_MIGRATION_HPP
