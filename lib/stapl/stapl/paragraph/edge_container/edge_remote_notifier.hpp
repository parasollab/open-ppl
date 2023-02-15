/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_EDGE_REMOTE_NOTIFIER_HPP
#define STAPL_PARAGRAPH_EDGE_REMOTE_NOTIFIER_HPP

#include <stapl/paragraph/edge_container/edge_container.h>
#include <stapl/paragraph/edge_container/utility.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Notifier passed to a producer location to notify a remote location
///   with only partial (filtered) or full value consumers.
/// @ingroup pgEdgeNotifiers
///
/// @tparam T Stored value type of associated edge.
///
/// @tparam Filter The filtering function that should be applied on the
/// producer location to avoid communicating portions of a produced value which
/// are not consumed at the destination location.
///
/// @todo For PARAGRAPHs (or even just a given edge) using full consumption (or
/// a statically defined list of filters), we could encode this information into
/// the type and avoid the member @p m_version_id, reducing communication
/// overhead on both the edge setup and subsequent data flow.
//////////////////////////////////////////////////////////////////////
template<typename T, typename Filter>
struct edge_remote_notifier
  : private Filter // filter stored as base for empty base optimization
{
private:
  typedef typename df_stored_type<T>::type             stored_value_t;

  /// @brief The version identifier for this value version of the task.
  /// Assigned by the remote consumer location. Passed back to @p receive_value
  /// which passes it to the @p edge_entry as an index into its versions list.
  unsigned int            m_version_id;

  // @brief Address of edge entry object for this value on the consuming
  /// location.  Send with the dataflow RMI to avoid container lookup.
  edge_entry_wrapper<T>   m_entry_wrapper;

public:
  edge_remote_notifier(Filter func,
                       const unsigned int version_id,
                       edge_entry_wrapper<T> entry_wrapper)
    : Filter(std::move(func)),
      m_version_id(version_id),
      m_entry_wrapper(std::move(entry_wrapper))
  { }

  void define_type(typer& t)
  {
    t.base<Filter>(*this);
    t.member(m_version_id);
    t.member(m_entry_wrapper);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke data flow reception on remote consumer location via ARMI.
  ///
  /// @param handle   The ARMI handle of the @p edge_container.
  /// @param loc      The remote consumer location.
  /// @param tid      The producer task identifier.
  /// @param version  The full produced value of task @p tid.  The notifier's
  ///                  filter will be applied prior to transmission.
  /// @todo Verify via assertion that non-moveable values are shareable.
  //////////////////////////////////////////////////////////////////////
  void operator()(rmi_handle::reference const& handle, size_t loc,
                  std::size_t tid,
                  edge_version_storage<stored_value_t> const& version,
                  bool b_moveable) const
  {
    const bool b_is_direct_storage = version.is_direct_storage();

    // If I can move, this takes precedence over immutable sharing
    // (the version storage here has no users and will be destroyed
    // after this notification).
    if (b_is_direct_storage && b_moveable)
    {
      async_rmi(loc, handle, &edge_container::receive_move_value<T, Filter>,
                tid, m_entry_wrapper,
                static_cast<Filter const&>(*this)
                  (std::move(const_cast<stored_value_t&>(version.value()))),
                 m_version_id);
      return;
    }

    // I can't move and this value isn't setup for sharing or is a filtered
    // consumer who will copy anyways. Just send a copy.
    if (b_is_direct_storage || m_version_id != 0)
    {
      async_rmi(loc, handle, &edge_container::receive_value<T, Filter>,
                tid, m_entry_wrapper,
                static_cast<Filter const&>(*this)(version.value()),
                m_version_id);
      return;
    }

    // else, share the value
    async_rmi(loc, handle, &edge_container::receive_shared_value<T, Filter>,
              tid, m_entry_wrapper, version.wrapper(), m_version_id);
  }
}; // struct edge_remote_notifier


//////////////////////////////////////////////////////////////////////
/// @brief Notifier passed to a producer location to notify a remote location
///   with only signal consumers.
/// @ingroup pgEdgeNotifiers
///
/// @tparam T Stored value type of associated edge.  In this context, always
/// @p int, and is used to match notifier operator() signature.  Ignored.
///
/// @sa edge_container::receive_signal
//////////////////////////////////////////////////////////////////////
template<typename T>
struct edge_remote_signal_notifier
{
  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke signal flow reception on remote consumer location via ARMI.
  ///
  /// @param handle  The ARMI handle of the @p edge_container.
  /// @param loc     The remote consumer location.
  /// @param tid     The producer task identifier.
  /// @param version Ignored stored edge value type.  Included to match
  ///                signature with @p edge_remote_notifier.
  //////////////////////////////////////////////////////////////////////
  void operator()(rmi_handle::reference const& handle, size_t loc,
                  std::size_t tid, edge_version_storage<T> const& version,
                  bool b_move) const
  {
    async_rmi(loc, handle, &edge_container::receive_signal, tid);
  }
};

} // namespace detail

} // namespace stapl

#endif // ifndef STAPL_PARAGRAPH_EDGE_REMOTE_NOTIFIER_HPP
