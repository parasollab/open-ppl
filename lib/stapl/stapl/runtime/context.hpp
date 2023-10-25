/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONTEXT_HPP
#define STAPL_RUNTIME_CONTEXT_HPP

#include "config.hpp"
#include "exception.hpp"
#include "this_context.hpp"
#include "instrumentation.hpp"
#include "location_md.hpp"
#include "message.hpp"
#include "runqueue.hpp"
#include "request/rmi_request.hpp"
#include "utility/logical_clock.hpp"
#include <iosfwd>
#include <limits>
#include <tuple>
#include <utility>
#include <boost/unordered_map.hpp>
#ifndef STAPL_RUNTIME_DISABLE_COMBINING
# include "request/rmi_delegate.hpp"
#endif

namespace stapl {

unsigned int get_aggregation(void) noexcept;


namespace runtime {

namespace context_impl {

//////////////////////////////////////////////////////////////////////
/// @brief RMI request aggregator to a destination location.
///
/// This class keeps an internal buffer where requests are aggregated and then
/// sent using one message.
///
/// If @ref STAPL_RUNTIME_DISABLE_COMBINING is not defined, it holds additional
/// information for combining requests (@see async_rmi).
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
class aggregator
{
public:
  using epoch_type = logical_clock::time_type;
private:
#ifndef STAPL_RUNTIME_DISABLE_COMBINING
  using pair_type  = std::pair<rmi_delegate, void*>;
#endif

  context&            m_context;
  /// Destination location.
  const full_location m_dest;
  /// Destination gang.
  gang_md* const      m_dest_g;
  /// @c true if the aggregator goes to a location on shared memory.
  const bool          m_on_shmem;
  /// Last known epoch.
  epoch_type          m_epoch;
  /// Request buffer.
  message_ptr         m_msg;
  /// Number of aggregated requests.
  unsigned int        m_aggregated;
#ifndef STAPL_RUNTIME_DISABLE_COMBINING
  /// Last request type and position in the buffer.
  pair_type           m_last_request;
#endif
  /// @c true if requests are intragang.
  const bool          m_intragang;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref aggregator to an intragang/intergang destination
  ///        with the gang metadata being known.
  ///
  /// If it is an intergang aggregator, then the reference count on @ref gang_md
  /// is increased to avoid deleting the @ref gang_md object in case that it is
  /// ready to be destroyed.
  //////////////////////////////////////////////////////////////////////
  aggregator(context& ctx,
             full_location const& dest,
             gang_md& g,
             const bool intragang) noexcept
  : m_context(ctx),
    m_dest(dest),
    m_dest_g(&g),
    m_on_shmem(
      g.get_process_id(dest.get_location_id())==runqueue::get_process_id()
    ),
    m_epoch(logical_clock::no_time),
    m_aggregated(0),
    m_intragang(intragang)
  {
    if (!intragang)
      g.add_ref();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref aggregator to an intergang destination.
  //////////////////////////////////////////////////////////////////////
  aggregator(context& ctx, full_location const& dest) noexcept
  : m_context(ctx),
    m_dest(dest),
    m_dest_g(nullptr),
    m_on_shmem(false),
    m_epoch(logical_clock::no_time),
    m_aggregated(0),
    m_intragang(false)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Destroys this @ref aggregator.
  ///
  /// The destructor flushes the buffer if necessary and releases the reference
  /// count on @ref gang_md if it was an intergang aggegrator.
  //////////////////////////////////////////////////////////////////////
  ~aggregator(void)
  {
    check_and_flush();
    if (!m_intragang && m_dest_g)
      m_dest_g->release();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if the destination is on shared memory, otherwise
  ///        @c false.
  //////////////////////////////////////////////////////////////////////
  bool is_on_shmem(void) const noexcept
  { return m_on_shmem; }

private:
  void allocate_message(const std::size_t, const bool);

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Allocates space for a new request.
  ///
  /// It will flush any aggregated requests if the new request is in a newer
  /// epoch than the last one or if there is not enough space for it.
  //////////////////////////////////////////////////////////////////////
  void* allocate(const std::size_t size,
                 const epoch_type e,
                 const bool implicit_flush)
  {
    if (m_msg) {
      if (e==m_epoch) {
        void* const p = m_msg->try_reserve(size);
        if (p) {
          ++m_aggregated;
#ifndef STAPL_RUNTIME_DISABLE_COMBINING
          m_last_request = pair_type{rmi_delegate{}, nullptr};
#endif
          return p;
        }
      }
      flush();
    }
    m_epoch      = e;
    m_aggregated = 1;
    allocate_message(size, implicit_flush);
#ifndef STAPL_RUNTIME_DISABLE_COMBINING
    m_last_request = pair_type{rmi_delegate{}, nullptr};
#endif
    return m_msg->reserve(size);
  }

#ifndef STAPL_RUNTIME_DISABLE_COMBINING

  //////////////////////////////////////////////////////////////////////
  /// @brief Allocates space for a new request that supports combining.
  ///
  /// It will flush any aggregated requests if the new request is in a newer
  /// epoch than the last one or if there is not enough space for it.
  ///
  /// @return The request that is used to combine the current one and a pointer
  ///         to available space.
  //////////////////////////////////////////////////////////////////////
  std::pair<void*, void*> allocate(rmi_delegate const& delegate,
                                   const std::size_t size,
                                   const std::size_t combined_size,
                                   const epoch_type e,
                                   const bool implicit_flush)
  {
    STAPL_RUNTIME_ASSERT(size>=combined_size);
    if (m_msg) {
      if (e==m_epoch) {
        if (m_last_request.first==delegate) {
          void* const p = m_msg->try_reserve(combined_size);
          if (p)
            return std::pair<void*, void*>{m_last_request.second, p};
        }
        else {
          void* const p = m_msg->try_reserve(size);
          if (p) {
            ++m_aggregated;
            m_last_request = pair_type{delegate, p};
            return std::pair<void*, void*>{nullptr, p};
          }
        }
      }
      flush();
    }
    m_epoch      = e;
    m_aggregated = 1;
    allocate_message(size, implicit_flush);
    void* const p  = m_msg->reserve(size);
    m_last_request = pair_type{delegate, p};
    return std::pair<void*, void*>{nullptr, p};
  }

#endif

  //////////////////////////////////////////////////////////////////////
  /// @brief Flushes the buffer.
  //////////////////////////////////////////////////////////////////////
  void flush(void)
  {
    STAPL_RUNTIME_ASSERT(m_msg);
    STAPL_RUNTIME_STATISTICS("aggregation", m_aggregated);
    if (m_dest_g)
      runqueue::add(*m_dest_g,
                    m_dest.get_location_id(),
                    m_on_shmem,
                    std::move(m_msg));
    else
      runqueue::forward(m_dest.get_gang_id(), std::move(m_msg));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Flushes the buffer if it has at least a request in it.
  //////////////////////////////////////////////////////////////////////
  void check_and_flush(void)
  {
    if (m_msg)
      flush();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Flushes the buffer if it has enough requests in it.
  ///
  /// @see get_aggregation()
  //////////////////////////////////////////////////////////////////////
  void try_flush(void)
  {
    if ((m_msg->available_space()<rmi_request::minimum_size()) ||
        (m_aggregated>=get_aggregation()))
      flush();
  }
};

} // namespace context_impl


//////////////////////////////////////////////////////////////////////
/// @brief Execution context of a task or request.
///
/// All tasks and requests execute within a context defined by a @ref context
/// object. Each context has a unique @ref context_id object that acts as its
/// id.
///
/// Contexts of a new execution environment are called base contexts. A base
/// context is not created by a request and therefore it has a nesting level of
/// @c 0 and mirrors the epoch of the associated @ref location_md object.
///
/// Every other context is created as the result of executing requests and they
/// have nesting level that is that of the request source + 1 and mirror the
/// epoch of the calling context.
///
/// Imposing an ordering between contexts is the backbone of request ordering.
/// Each @ref context object is associated with a queue of requests which will
/// execute in a FIFO order. Therefore, changing the ordering of @ref context
/// objects changes the request execution order.
///
/// @see context_id, gang_md, location_md
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
class context
{
public:
  using id              = context_id;
  using aggregator_type = context_impl::aggregator;
  using epoch_type      = logical_clock::time_type;
private:
  using aggr_map_type   = boost::unordered_map<full_location, aggregator_type>;

  // context information

  /// Context id.
  const id                       m_id;
  /// Associated location metadata.
  location_md&                   m_location;
  /// Last request epoch.
  epoch_type                     m_last_epoch;

  // replies

  /// Source process id.
  const process_id               m_src_pid;
  /// Return value buffer.
  message_ptr                    m_ret_msg;

  // aggregators

  /// Request aggregators per destination location.
  aggr_map_type                  m_aggr;
  aggr_map_type::iterator        m_last_aggr;

  // sent / processed metadata

  /// Sent intragang buffers.
  unsigned int                   m_sent;
  /// Processed buffers.
  unsigned int                   m_proc;
  boost::intrusive_ptr<fence_md> m_intergang_md;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new @ref context object from the given id and
  ///        associates it with the metadata.
  //////////////////////////////////////////////////////////////////////
  context(id const& cid,
          location_md& l,
          const process_id src_pid)
  : m_id(cid),
    m_location(l),
    m_last_epoch(logical_clock::start_time),
    m_src_pid(src_pid),
    m_last_aggr(m_aggr.end()),
    m_sent(0),
    m_proc(0)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new @ref context object from the given metadata.
  ///
  /// This is base context, meaning that it executes code that was not triggered
  /// by a request. It will be pushed on the stack automatically.
  //////////////////////////////////////////////////////////////////////
  explicit context(location_md& l)
  : m_id(full_location(l.get_gang_md().get_id(), l.get_id())),
    m_location(l),
    m_last_epoch(logical_clock::start_time),
    m_src_pid(invalid_process_id),
    m_last_aggr(m_aggr.end()),
    m_sent(0),
    m_proc(0)
  { this_context::push_base(*this); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Destroys this @ref context object.
  ///
  /// All requests and metadata will be flushed. In the case that it was a base
  /// context, then it will be removed from the stack automatically.
  //////////////////////////////////////////////////////////////////////
  ~context(void)
  {
    if (is_base()) {
      // this base context has finished execution
      flush();
      this_context::pop_base(*this);
    }
    STAPL_RUNTIME_ASSERT(!m_ret_msg && m_aggr.empty() &&
                         (m_sent==0) && (m_proc==0) && !m_intergang_md);
  }

  context(context const&) = delete;
  context& operator=(context const&) = delete;

  id const& get_id(void) const noexcept
  { return m_id; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the gang id and location id as a @ref full_location object
  ///        that created this @ref context object.
  //////////////////////////////////////////////////////////////////////
  full_location const& get_initiator(void) const noexcept
  { return m_id.initiator; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the gang id and location id associated with this
  ///        @ref context as a @ref full_location object.
  //////////////////////////////////////////////////////////////////////
  full_location const& get_current_location(void) const noexcept
  { return m_id.current; }

  gang_md const& get_gang_md(void) const noexcept
  { return m_location.get_gang_md(); }

  gang_md& get_gang_md(void) noexcept
  { return m_location.get_gang_md(); }

  location_md const& get_location_md(void) const noexcept
  { return m_location; }

  location_md& get_location_md(void) noexcept
  { return m_location; }

  gang_md::id get_gang_id(void) const noexcept
  { return m_id.current.get_gang_id(); }

  location_md::id get_location_id(void) const noexcept
  { return m_id.current.get_location_id(); }

  process_id get_source_process_id(void) const noexcept
  { return m_src_pid; }

  bool is_intragang(void) const noexcept
  { return m_id.intragang; }

  nesting_level get_nesting(void) const noexcept
  { return m_id.nesting; }

  bool is_base(void) const noexcept
  { return (get_nesting()==0); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets the epoch this @ref context is in.
  ///
  /// If epoch differs from the previous, requests will be flushed.
  //////////////////////////////////////////////////////////////////////
  void set_epoch(const epoch_type e)
  {
    STAPL_RUNTIME_ASSERT(!is_base());
    if (m_last_epoch!=e) {
      flush_requests();
      m_last_epoch = e;
    }
  }

  epoch_type get_epoch(void) const noexcept
  { return (is_base() ? get_location_md().get_epoch() : m_last_epoch); }

// --------------------------------------------------------------------------
// Requests
// --------------------------------------------------------------------------

  //////////////////////////////////////////////////////////////////////
  /// @brief Allocates space for a return value request.
  //////////////////////////////////////////////////////////////////////
  void* allocate_return(const std::size_t size)
  {
    STAPL_RUNTIME_ASSERT(!is_base());
    if (m_ret_msg) {
      void* const p = m_ret_msg->try_reserve(size);
      if (p)
        return p;
      runqueue::add(m_src_pid, std::move(m_ret_msg)); // buffer too small
    }
    m_ret_msg = message::create(header::RPC, size);
    return m_ret_msg->reserve(size);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a request aggregator to the given destination.
  ///
  /// The aggregator from the previous call to @ref get_aggregator() is memoized
  /// unless @ref flush() has been called.
  ///
  /// @warning While memoization may have a positive effect when a location
  ///          communicates with the same location repeatedly, it may have no or
  ///          negative performance effects under a less repeated communication
  ///          pattern.
  ///
  /// @warning If the gang metadata is not known, requests are sent to the owner
  ///          process of the destination gang id, which always knows where to
  ///          forward them. This can potentially create a choking point.
  //////////////////////////////////////////////////////////////////////
  aggregator_type& get_aggregator(full_location const& loc)
  {
    // check if loc was the last location to sent a request to
    if (m_last_aggr != m_aggr.end() && m_last_aggr->first == loc)
      return m_last_aggr->second;

    if (loc.get_gang_id()==get_gang_id()) {
      // source and destination locations are on same gang, thus metadata known
      m_last_aggr = m_aggr.emplace(
                      std::piecewise_construct,
                      std::forward_as_tuple(loc),
                      std::forward_as_tuple(
                        *this, loc, std::ref(get_gang_md()), is_intragang())
                    ).first;
      return m_last_aggr->second;
    }

    // different gang aggregator

    auto it = m_aggr.find(loc);
    if (it!=m_aggr.end()) {
      // aggregator already created
      m_last_aggr = it;
      return m_last_aggr->second;
    }

    auto* const g = get_location_md().get_cached_gang_md(loc.get_gang_id());
    if (g) {
      // Gang metadata is known. Requests are sent to the process where the
      // destination location is.
      m_last_aggr = m_aggr.emplace(
                      std::piecewise_construct,
                      std::forward_as_tuple(loc),
                      std::forward_as_tuple(*this, loc, std::ref(*g), false)
                    ).first;
      return m_last_aggr->second;
    }

    // Gang metadata is not known. Requests are sent to the gang id owner which
    // in turn will forward them to the destination location. This can harm
    // performance, since it creates a serialization point. However, it is
    // probably not a very frequent occurrence.
    m_last_aggr = m_aggr.emplace(std::piecewise_construct,
                                 std::forward_as_tuple(loc),
                                 std::forward_as_tuple(*this, loc)).first;
    return m_last_aggr->second;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Flushes all aggregated requests.
  //////////////////////////////////////////////////////////////////////
  void flush_requests(void)
  {
    m_aggr.clear(); // destructor sends the requests
    m_last_aggr = m_aggr.end();
  }

// --------------------------------------------------------------------------
// Fence metadata
// --------------------------------------------------------------------------

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Flushes metadata required for quiescence detection (i.e. fence).
  ///
  /// Fence metadata for intragang requests are written directly to the
  /// associated location metadata object.
  ///
  /// Fence metadata for intergang requests are written to the associated gang
  /// metadata object if present, otherwise they are forwarded to the gang id
  /// owner. This may create additional traffic at the gang id owner.
  //////////////////////////////////////////////////////////////////////
  void flush_fence_md(void);

  //////////////////////////////////////////////////////////////////////
  /// @brief Increases the count of pending intergang buffers to gang @p gid by
  ///        @p N.
  //////////////////////////////////////////////////////////////////////
  void count_intergang_pending(const gang_md::id gid, const unsigned int N);

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Flushes all aggregated requests and fence metadata.
  //////////////////////////////////////////////////////////////////////
  void flush(void)
  {
    flush_requests();
    flush_fence_md();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Increases the count of pending buffers.
  ///
  /// This function is called when the destination of the requests in the buffer
  /// is @p loc.
  //////////////////////////////////////////////////////////////////////
  void count_pending(full_location const& loc)
  {
    const auto dst_gid = loc.get_gang_id();
    if (dst_gid==get_gang_id())
      ++m_sent;
    else
      count_intergang_pending(dst_gid, 1);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Increases the count of pending buffers by @p N.
  ///
  /// This function is called when the destination of the requests in the buffer
  /// is some or all locations in gang @p dst_gid.
  //////////////////////////////////////////////////////////////////////
  void count_pending(const gang_md::id dst_gid, const unsigned int N)
  {
    if (dst_gid==get_gang_id())
      m_sent += N;
    else
      count_intergang_pending(dst_gid, N);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Increases the count of processed buffers.
  ///
  /// This function flushes the buffer containing return values, since blocking
  /// calls may be waiting for them.
  //////////////////////////////////////////////////////////////////////
  void count_processed(void)
  {
    STAPL_RUNTIME_ASSERT(!is_base());
    if (m_ret_msg)
      runqueue::add(m_src_pid, std::move(m_ret_msg));
    ++m_proc;
  }
};


namespace context_impl {

// Allocates a message
inline void
aggregator::allocate_message(const std::size_t size, const bool implicit_flush)
{
  if (m_context.get_nesting()==std::numeric_limits<nesting_level>::max())
    STAPL_RUNTIME_ERROR("Too many nested RMI invocations.");

  m_context.count_pending(m_dest);

  const nesting_level nesting = (m_context.get_nesting() + 1);

  context_id dst_cid{m_context.get_initiator(),
                     m_dest,
                     m_context.get_current_location(),
                     m_intragang,
                     nesting,
                     0}; // unknown magic number

  if (nesting>2) {
    dst_cid.magic =
      m_context.get_location_md().disambiguate(m_context.get_id(), dst_cid);
  }

  m_msg = message::create((m_dest_g ? header::RMI : header::FWD_RMI),
                          size,
                          header::request{std::move(dst_cid), m_epoch},
                          implicit_flush);
}

} // namespace context_impl


std::ostream& operator<<(std::ostream&, context const&);

} // namespace runtime

} // namespace stapl

#endif
