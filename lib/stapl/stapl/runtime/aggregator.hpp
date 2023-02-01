/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_AGGREGATOR_HPP
#define STAPL_RUNTIME_AGGREGATOR_HPP

#include "context.hpp"
#include "rmi_handle_fwd.hpp"
#include "runqueue.hpp"
#include "synchronization.hpp"
#include "tags.hpp"
#include <iterator>
#include <utility>
#include <vector>
#include <boost/range/counting_range.hpp>
#ifndef STAPL_RUNTIME_DISABLE_COMBINING
# include "request/rmi_delegate.hpp"
#endif

namespace stapl {

namespace runtime {

////////////////////////////////////////////////////////////////////
/// @brief Returns the epoch that a request from context @p ctx on handle @p h
///        should execute in.
///
/// @param ctx Current @ref context.
/// @param h   Destination object handle.
///
/// @ingroup requestBuildingBlock
////////////////////////////////////////////////////////////////////
template<typename Handle>
context::epoch_type
find_target_epoch(context const& ctx, Handle const& h) noexcept
{
  if (ctx.is_intragang() && (ctx.get_gang_id()==h.get_gang_id())) {
    // intragang request; the epoch of the context is more important than that
    // of the object
    return ctx.get_epoch();
  }

  // intergang request; we can only trust the epoch of the object
  return h.get_epoch();
}


////////////////////////////////////////////////////////////////////
/// @brief Returns if possible the gang metadata associated with @p h, otherwise
///        @c nullptr.
///
/// @param ctx Current @ref context.
/// @param h   Destination object handle.
///
/// @ingroup requestBuildingBlock
////////////////////////////////////////////////////////////////////
template<typename Handle>
gang_md* find_target_gang_md(context& ctx, Handle const& h) noexcept
{
  const auto gid = h.get_gang_id();
  return (ctx.get_gang_id()==gid ? &(ctx.get_gang_md())
                                 : gang_md_registry::try_get(gid));
}


// --------------------------------------------------------------------------
// RMIs
// --------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////
/// @brief Aggregator for point-to-point requests.
///
/// It aggregates requests in an internal buffer until the latter is flushed,
/// either because it is full or because of an explicit flush.
///
/// @ingroup requestBuildingBlock
///
/// @todo This class needs a better name. Although it allows aggregation and
///       combining, it does not do it on its own. What it does is to find
///       space to construct a request in the @ref context object.
////////////////////////////////////////////////////////////////////
class aggregator
{
private:
  context::aggregator_type& m_aggregator;
  const context::epoch_type m_epoch;
  const bool                m_flush;

public:
  ////////////////////////////////////////////////////////////////////
  /// @brief Create a new @ref aggregator that flushes requests by default when
  ///        destroyed.
  ///
  /// @param ctx Current @ref context.
  /// @param h   Destination object handle.
  /// @param dst Destination location.
  ////////////////////////////////////////////////////////////////////
  template<typename Handle>
  aggregator(context& ctx,
             Handle const& h,
             const location_md::id dst)
  : m_aggregator(ctx.get_aggregator(full_location{h.get_gang_id(), dst})),
    m_epoch(find_target_epoch(ctx, h)),
    m_flush(true)
  { STAPL_RUNTIME_ASSERT(h.valid() && h.is_valid(dst)); }

  ////////////////////////////////////////////////////////////////////
  /// @brief Create a new @ref aggregator that may or may not flush requests
  ///        when destroyed, depending on the handle flags.
  ///
  /// @param ctx Current @ref context.
  /// @param h   Destination object handle.
  /// @param dst Destination location.
  ////////////////////////////////////////////////////////////////////
  template<typename Handle>
  aggregator(context& ctx,
             Handle const& h,
             const location_md::id dst,
             no_implicit_flush_t)
  : m_aggregator(ctx.get_aggregator(full_location{h.get_gang_id(), dst})),
    m_epoch(find_target_epoch(ctx, h)),
    m_flush((h.get_flags() & stapl::no_aggregation)!=0)
  { STAPL_RUNTIME_ASSERT(h.valid() && h.is_valid(dst)); }

  ~aggregator(void)
  {
    if (m_flush)
      m_aggregator.flush();
    else
      m_aggregator.try_flush();
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if the destination is on shared memory.
  ////////////////////////////////////////////////////////////////////
  bool is_on_shmem(void) const noexcept
  { return m_aggregator.is_on_shmem(); }

  ////////////////////////////////////////////////////////////////////
  /// @brief Allocates @p n bytes for storing a request.
  ////////////////////////////////////////////////////////////////////
  void* allocate(const std::size_t n)
  { return m_aggregator.allocate(n, m_epoch, m_flush); }

#if !defined(STAPL_RUNTIME_DISABLE_COMBINING)

  ////////////////////////////////////////////////////////////////////
  /// @brief Allocates @p combined bytes for storing a combined request or @p n
  ///        bytes if combining is not possible.
  ///
  /// @return An @c std::pair with the request that the new one will be combined
  ///         with and a pointer to available space.
  ////////////////////////////////////////////////////////////////////
  std::pair<void*, void*> allocate(rmi_delegate const& d,
                                   const std::size_t n,
                                   const std::size_t combined)
  { return m_aggregator.allocate(d, n, combined, m_epoch, m_flush); }

#endif
};


////////////////////////////////////////////////////////////////////
/// @brief Aggregator for ordered and unordered broadcast requests.
///
/// If an ordered broadcast is requested, then all previously aggregated
/// requests are flushed, otherwise they are not.
///
/// @ingroup requestBuildingBlock
////////////////////////////////////////////////////////////////////
class bcast_aggregator
{
private:
  context&                  m_context;
  const gang_md::id         m_dst_gid;
  gang_md*                  m_dst_g;
  const unsigned int        m_num_locs;
  const context::epoch_type m_epoch;
  const bool                m_ordered;
  message_ptr               m_msg;

public:
  ////////////////////////////////////////////////////////////////////
  /// @brief Create a new @ref bcast_aggregator.
  ///
  /// @param ctx     Current @ref context.
  /// @param h       Destination object handle.
  /// @param ordered @c true if the request is ordered, otherwise @c false.
  ////////////////////////////////////////////////////////////////////
  template<typename Handle>
  bcast_aggregator(context& ctx, Handle const& h, const bool ordered = true)
  : m_context(ctx),
    m_dst_gid(h.get_gang_id()),
    m_dst_g(nullptr),
    m_num_locs(h.get_num_locations()),
    m_epoch(find_target_epoch(ctx, h)),
    m_ordered(ordered)
  {
    STAPL_RUNTIME_ASSERT(h.valid());
    if (m_ordered)
      m_context.flush_requests();

    if (m_context.get_gang_id()==m_dst_gid) {
      m_dst_g = &(m_context.get_gang_md());
    }
    else {
      m_dst_g = (m_ordered
                  ? m_context.get_location_md().get_cached_gang_md(m_dst_gid)
                  : gang_md_registry::try_get(m_dst_gid));
    }
  }

  ~bcast_aggregator(void)
  {
    if (m_dst_g)
      runqueue::add_all(*m_dst_g, m_ordered, std::move(m_msg));
    else
      runqueue::forward(m_dst_gid, std::move(m_msg));
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Allocates @p n bytes for storing a request.
  ////////////////////////////////////////////////////////////////////
  void* allocate(const std::size_t n)
  {
    m_context.count_pending(m_dst_gid, m_num_locs);

    const auto hdr = (m_ordered ? (m_dst_g ? header::BCAST_RMI
                                           : header::FWD_BCAST_RMI)
                                : (m_dst_g ? header::UNORDERED_BCAST_RMI
                                           : header::FWD_UNORDERED_BCAST_RMI));
    const bool intragang =
      (m_context.is_intragang() & (m_context.get_gang_id()==m_dst_gid));
    const nesting_level nesting = (m_context.get_nesting() + 1);

    if (nesting <= 2) {
      m_msg = message::create(hdr,
                              n,
                              header::bcast_request{
                                m_context.get_initiator(),
                                m_dst_gid,
                                m_context.get_current_location(),
                                intragang,
                                nesting,
                                m_epoch
                              });
    }
    else {
      context_id cid{m_context.get_initiator(),
                     m_dst_gid,
                     m_context.get_current_location(),
                     intragang,
                     nesting,
                     0}; // unknown magic number
      cid.magic = m_context.get_location_md().disambiguate(m_context.get_id(),
                                                           cid);
      m_msg = message::create(hdr,
                              n,
                              header::bcast_request{std::move(cid), m_epoch});
    }

    return m_msg->reserve(n);
  }
};


// --------------------------------------------------------------------------
// RPCs
// --------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////
/// @brief Aggregator for RPC requests to a process or a range of processes.
///
/// It flushes requests by default when destroyed.
///
/// @ingroup requestBuildingBlock
////////////////////////////////////////////////////////////////////
class rpc_aggregator
{
private:
  const process_id           m_dest;
  runqueue::process_id_range m_dests;
  message_ptr                m_msg;

public:
  ////////////////////////////////////////////////////////////////////
  /// @brief Create a new @ref rpc_aggregator.
  ///
  /// @param dest Destination process.
  ////////////////////////////////////////////////////////////////////
  explicit rpc_aggregator(const process_id dest) noexcept
  : m_dest(dest)
  { }

  ////////////////////////////////////////////////////////////////////
  /// @brief Create a new @ref rpc_aggregator.
  ///
  /// @param dests Range of destination processes.
  ////////////////////////////////////////////////////////////////////
  template<typename Range>
  explicit rpc_aggregator(Range&& r)
  : m_dest(invalid_process_id),
    m_dests(std::forward<Range>(r))
  { STAPL_RUNTIME_ASSERT(!m_dests.empty()); }

  ~rpc_aggregator(void)
  {
    if (m_dests.empty())
      runqueue::add(m_dest, std::move(m_msg));
    else
      runqueue::add_all(std::move(m_dests), std::move(m_msg));
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Allocates @p n bytes for storing a request.
  ////////////////////////////////////////////////////////////////////
  void* allocate(const std::size_t n)
  {
    m_msg = message::create(header::RPC, n);
    return m_msg->reserve(n);
  }
};


////////////////////////////////////////////////////////////////////
/// @brief Aggregator for RPC requests to a single location.
///
/// It flushes requests by default when destroyed.
///
/// @ingroup requestBuildingBlock
////////////////////////////////////////////////////////////////////
class location_rpc_aggregator
{
private:
  const gang_md::id         m_dst_gid;
  gang_md*                  m_dst_g;
  const location_md::id     m_dst_lid;
  const context::epoch_type m_epoch;
  message_ptr               m_msg;

public:
  ////////////////////////////////////////////////////////////////////
  /// @brief Create a new @ref location_rpc_aggregator.
  ///
  /// @param g     Destination gang.
  /// @param lid   Destination location.
  /// @param epoch Current epoch.
  ////////////////////////////////////////////////////////////////////
  location_rpc_aggregator(gang_md& g,
                          const location_md::id lid,
                          const context::epoch_type epoch) noexcept
  : m_dst_gid(g.get_id()),
    m_dst_g(&g),
    m_dst_lid(lid),
    m_epoch(epoch)
  { }

  ////////////////////////////////////////////////////////////////////
  /// @brief Create a new @ref location_rpc_aggregator.
  ///
  /// @param gid   Destination gang id.
  /// @param lid   Destination location.
  /// @param epoch Current epoch.
  ////////////////////////////////////////////////////////////////////
  location_rpc_aggregator(const gang_md::id gid,
                          const location_md::id lid,
                          const context::epoch_type epoch) noexcept
  : m_dst_gid(gid),
    m_dst_g(gang_md_registry::try_get(gid)),
    m_dst_lid(lid),
    m_epoch(epoch)
  { }

  ~location_rpc_aggregator(void)
  {
    if (m_dst_g) {
      const auto on_shmem =
        (runqueue::get_process_id()==m_dst_g->get_process_id(m_dst_lid));
      runqueue::add(*m_dst_g, m_dst_lid, on_shmem, std::move(m_msg));
    }
    else {
      runqueue::forward(m_dst_gid, std::move(m_msg));
    }
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Allocates @p n bytes for storing a request.
  ////////////////////////////////////////////////////////////////////
  void* allocate(const std::size_t n)
  {
    const auto hdr = (m_dst_g ? header::LOCATION_RPC
                              : header::FWD_LOCATION_RPC);
    m_msg = message::create(hdr,
                            n,
                            header::location_rpc{
                              full_location{m_dst_gid, m_dst_lid}, m_epoch
                            });
    return m_msg->reserve(n);
  }
};


////////////////////////////////////////////////////////////////////
/// @brief Aggregator for RPC requests to all locations of a gang.
///
/// It flushes requests by default when destroyed.
///
/// @ingroup requestBuildingBlock
////////////////////////////////////////////////////////////////////
class all_locations_rpc_aggregator
{
private:
  const gang_md::id         m_dst_gid;
  gang_md* const            m_dst_g;
  const context::epoch_type m_epoch;
  message_ptr               m_msg;

public:
  ////////////////////////////////////////////////////////////////////
  /// @brief Create a new @ref all_locations_rpc_aggregator.
  ///
  /// @param gid   Destination gang id.
  /// @param epoch Current epoch.
  ////////////////////////////////////////////////////////////////////
  all_locations_rpc_aggregator(const gang_md::id gid,
                               const context::epoch_type epoch) noexcept
  : m_dst_gid(gid),
    m_dst_g(gang_md_registry::try_get(gid)),
    m_epoch(epoch)
  { }

  ////////////////////////////////////////////////////////////////////
  /// @brief Create a new @ref all_locations_rpc_aggregator.
  ///
  /// @param g     Destination gang.
  /// @param epoch Current epoch.
  ////////////////////////////////////////////////////////////////////
  all_locations_rpc_aggregator(gang_md& g,
                               const context::epoch_type epoch) noexcept
  : m_dst_gid(g.get_id()),
    m_dst_g(&g),
    m_epoch(epoch)
  { }

  ~all_locations_rpc_aggregator(void)
  {
    if (m_dst_g)
      runqueue::add_all(*m_dst_g, false, std::move(m_msg));
    else
      runqueue::forward(m_dst_gid, std::move(m_msg));
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Allocates @p n bytes for storing a request.
  ////////////////////////////////////////////////////////////////////
  void* allocate(const std::size_t n)
  {
    m_msg = message::create(header::BCAST_LOCATION_RPC,
                            n,
                            header::bcast_location_rpc{m_dst_gid, m_epoch});
    return m_msg->reserve(n);
  }
};


////////////////////////////////////////////////////////////////////
/// @brief Aggregator for RPC requests to all locations in shared memory.
///
/// It flushes requests by default when destroyed.
///
/// @ingroup requestBuildingBlock
////////////////////////////////////////////////////////////////////
class managed_locations_rpc_aggregator
{
private:
  gang_md&                    m_dst_g;
  const context::epoch_type   m_epoch;
  runqueue::location_id_range m_range;
  message_ptr                 m_msg;

public:
  ////////////////////////////////////////////////////////////////////
  /// @brief Create a new @ref managed_locations_rpc_aggregator.
  ///
  /// It sends the requests to all locally managed locations.
  ///
  /// @param g     Destination gang.
  /// @param epoch Current epoch.
  ////////////////////////////////////////////////////////////////////
  managed_locations_rpc_aggregator(gang_md& g,
                                   const context::epoch_type epoch) noexcept
  : m_dst_g(g),
    m_epoch(epoch)
  { }

  ////////////////////////////////////////////////////////////////////
  /// @brief Create a new @ref managed_locations_rpc_aggregator.
  ///
  /// @param g     Destination gang.
  /// @param r     Locally managed destination locations.
  /// @param epoch Current epoch.
  ////////////////////////////////////////////////////////////////////
  template<typename Range>
  managed_locations_rpc_aggregator(gang_md& g,
                                   Range&& r,
                                   const context::epoch_type epoch) noexcept
  : m_dst_g(g),
    m_epoch(epoch),
    m_range(std::forward<Range>(r))
  { STAPL_RUNTIME_ASSERT(!m_range.empty()); }

  ~managed_locations_rpc_aggregator(void)
  {
    runqueue::add_managed(m_dst_g, std::move(m_range), false, std::move(m_msg));
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Allocates @p n bytes for storing a request.
  ////////////////////////////////////////////////////////////////////
  void* allocate(const std::size_t n)
  {
    m_msg = message::create(header::BCAST_LOCATION_RPC,
                            n,
                            header::bcast_location_rpc{
                              m_dst_g.get_id(), m_epoch
                            });
    return m_msg->reserve(n);
  }
};


// --------------------------------------------------------------------------
// Responses
// --------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////
/// @brief Aggregator for responses to requests.
///
/// It aggregates responses in an internal buffer until the latter is flushed,
/// either because it is full or because of an explicit flush.
///
/// @ingroup requestBuildingBlock
////////////////////////////////////////////////////////////////////
class response_aggregator
{
private:
  context& m_context;

public:
  constexpr explicit response_aggregator(context& ctx) noexcept
  : m_context(ctx)
  { }

  ////////////////////////////////////////////////////////////////////
  /// @brief Allocates @p n bytes for storing a response.
  ////////////////////////////////////////////////////////////////////
  void* allocate(const std::size_t n)
  { return m_context.allocate_return(n); }
};

} // namespace runtime

} // namespace stapl

#endif
