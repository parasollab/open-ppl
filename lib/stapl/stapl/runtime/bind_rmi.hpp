/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_BIND_RMI_HPP
#define STAPL_RUNTIME_BIND_RMI_HPP

#include "aggregator.hpp"
#include "context.hpp"
#include "request/async_rmi_request.hpp"
#include "type_traits/transport_qualifier.hpp"
#include "utility/unused.hpp"
#include <functional>
#include <limits>
#include <type_traits>
#include <unordered_map>
#include <utility>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief RMI request aggregator for a single destination location.
///
/// This class keeps an internal buffer where requests are stored and then sent
/// as one message.
///
/// @ingroup runtimeMetadata
///
/// @todo This class needs to be combined with @ref context_impl::aggregator.
//////////////////////////////////////////////////////////////////////
class tunnel_aggregator
{
public:
  using epoch_type = logical_clock::time_type;

private:
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
  /// Last request type and position in the buffer.
  void*               m_last_request;
  /// @c true if requests are intragang.
  const bool          m_intragang;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref tunnel_aggregator to an intragang/intergang
  ///        destination with the gang metadata being known.
  ///
  /// If it is an intergang aggregator, then the reference count on @ref gang_md
  /// is increased to avoid deleting the @ref gang_md object in case that it is
  /// ready to be destroyed.
  //////////////////////////////////////////////////////////////////////
  tunnel_aggregator(context& ctx,
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
    m_last_request(nullptr),
    m_intragang(intragang)
  {
    if (!intragang)
      g.add_ref();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref tunnel_aggregator to an intergang destination.
  //////////////////////////////////////////////////////////////////////
  tunnel_aggregator(context& ctx, full_location const& dest) noexcept
  : m_context(ctx),
    m_dest(dest),
    m_dest_g(nullptr),
    m_on_shmem(false),
    m_epoch(logical_clock::no_time),
    m_last_request(nullptr),
    m_intragang(false)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Destroys this @ref tunnel_aggregator.
  ///
  /// The destructor flushes the buffer if necessary and releases the reference
  /// count on @ref gang_md if it was an intergang aggegrator.
  //////////////////////////////////////////////////////////////////////
  ~tunnel_aggregator(void)
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
  void allocate_message(const std::size_t size)
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
                            false);
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Allocates space for a new request that supports combining.
  ///
  /// It will flush any aggregated requests if the new request is in a newer
  /// epoch than the last one or if there is not enough space for it.
  ///
  /// @return The request that is used to combine the current one and a pointer
  ///         to available space.
  //////////////////////////////////////////////////////////////////////
  std::pair<void*, void*> allocate(const std::size_t size,
                                   const std::size_t combined_size,
                                   const epoch_type e)
  {
    STAPL_RUNTIME_ASSERT(size>=combined_size);
    if (m_msg) {
      if (e==m_epoch) {
        void* const p = m_msg->try_reserve(combined_size);
        if (p)
          return std::pair<void*, void*>{m_last_request, p};
      }
      flush();
    }
    m_epoch = e;
    allocate_message(size);
    void* const p  = m_msg->reserve(size);
    m_last_request = p;
    return std::pair<void*, void*>{nullptr, p};
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Flushes the buffer.
  //////////////////////////////////////////////////////////////////////
  void flush(void)
  {
    STAPL_RUNTIME_ASSERT(m_msg);
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
    if (m_msg->available_space()<rmi_request::minimum_size())
      flush();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Establishes a communication tunnel to a member function of a
///        distributed object.
///
/// @tparam R                            Member function result type.
/// @tparam T                            Object type.
/// @tparam MemberFunctionPointerCreator Member function pointer creator type.
/// @tparam Arg                          Member function argument types.
///
///
/// @ingroup runtimeMetadata
///
/// @todo Automatically flush metadata upon call of @ref rmi_fence().
//////////////////////////////////////////////////////////////////////
template<typename R,
         typename T,
         template<typename...> class MemberFunctionPointerCreator,
         typename... Arg>
class bind_rmi_result
{
private:
  using aggregator_type = tunnel_aggregator;
  using aggregator_container_type =
    std::unordered_map<full_location, aggregator_type>;
  using pmf_type = typename MemberFunctionPointerCreator<R, T, Arg...>::type;

  rmi_handle::reference     m_handle;
  pmf_type                  m_pmf;
  context&                  m_ctx;
  aggregator_container_type m_aggr;

public:
  bind_rmi_result(pmf_type const& pmf, rmi_handle::reference const& h)
  : m_handle{h},
    m_pmf{pmf},
    m_ctx(this_context::get())
  { STAPL_RUNTIME_ASSERT_MSG(h.valid(), "Invalid handle"); }

private:
  gang_md const& get_gang_md(void) const noexcept
  { return m_ctx.get_gang_md(); }

  gang_md& get_gang_md(void) noexcept
  { return m_ctx.get_gang_md(); }

  location_md const& get_location_md(void) const noexcept
  { return m_ctx.get_location_md(); }

  location_md& get_location_md(void) noexcept
  { return m_ctx.get_location_md(); }

  bool is_intragang(void) const noexcept
  { return m_ctx.is_intragang(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a request aggregator to the given destination.
  ///
  /// @warning If the gang metadata is not known, requests are sent to the owner
  ///          process of the destination gang id, which always knows where to
  ///          forward them. This can potentially create a choking point.
  //////////////////////////////////////////////////////////////////////
  aggregator_type& get_aggregator(full_location const& loc)
  {
    if (loc.get_gang_id()==get_gang_md().get_id()) {
      // source and destination locations are on same gang, thus metadata known
      return m_aggr.emplace(
               std::piecewise_construct,
               std::forward_as_tuple(loc.get_location_id()),
               std::forward_as_tuple(
                 std::ref(m_ctx), loc, std::ref(get_gang_md()), is_intragang())
               ).first->second;
    }

    // different gang aggregator

    auto it = m_aggr.find(loc);
    if (it!=m_aggr.end()) {
      // aggregator already created
      return it->second;
    }

    auto* const g = get_location_md().get_cached_gang_md(loc.get_gang_id());
    if (g) {
      // Gang metadata is known. Requests are sent to the process where the
      // destination location is.
      return m_aggr.emplace(
               std::piecewise_construct,
               std::forward_as_tuple(loc),
               std::forward_as_tuple(std::ref(m_ctx), loc, std::ref(*g), false)
             ).first->second;
    }

    // Gang metadata is not known. Requests are sent to the gang id owner which
    // in turn will forward them to the destination location. This can harm
    // performance, since it creates a serialization point. However, it is
    // probably not a very frequent occurrence.
    return m_aggr.emplace(
             std::piecewise_construct,
             std::forward_as_tuple(loc),
             std::forward_as_tuple(std::ref(m_ctx), loc)
           ).first->second;
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Flushes all pending requests.
  //////////////////////////////////////////////////////////////////////
  void flush(void)
  {
    m_aggr.clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sends a request to location @p lid.
  //////////////////////////////////////////////////////////////////////
  template<typename... U>
  void operator()(const unsigned int lid, U&&... u)
  {
    auto& a = get_aggregator(full_location{m_handle.get_gang_id(), lid});

    const bool on_shmem = a.is_on_shmem();
    if (on_shmem) {
      using request_type =
        async_rmi_request<packed_handle_type,
                          pmf_type,
                          typename transport_qualifier<Arg>::type...>;

      const auto size = request_type::expected_size(std::forward<U>(u)...);

      auto p = a.allocate(size.non_combined_size(),
                          size.combined_size(),
                          find_target_epoch(m_ctx, m_handle));
      if (p.first) {
        // combine requests
        using args_type = typename request_type::args_type;

        static_cast<request_type*>(p.first)->combined(size.combined_size());
        auto sz = size.combined_static_size();
        unused(sz); // if sizeof...(U)==0, sz is unused
        new(p.second) args_type{std::forward_as_tuple(std::forward<U>(u),
                                                      p.second,
                                                      sz)...};
      }
      else {
        // no combining possible
        new(p.second) request_type{m_handle, m_pmf, std::forward<U>(u)...};
      }
    }
    else {
      using request_type =
        async_rmi_request<packed_handle_type,
                          pmf_type,
                          typename std::remove_reference<Arg>::type...>;

      const auto size = request_type::expected_size(std::forward<U>(u)...);

      auto p = a.allocate(size.non_combined_size(),
                          size.combined_size(),
                          find_target_epoch(m_ctx, m_handle));
      if (p.first) {
        // combine requests
        using args_type = typename request_type::args_type;

        static_cast<request_type*>(p.first)->combined(size.combined_size());
        auto sz = size.combined_static_size();
        unused(sz); // if sizeof...(U)==0, sz is unused
        new(p.second) args_type{std::forward_as_tuple(std::forward<U>(u),
                                                      p.second,
                                                      sz)...};
      }
      else {
        // no combining possible
        new(p.second) request_type{m_handle, m_pmf, std::forward<U>(u)...};
      }
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Returns a member function pointer of type <tt>R (T::*)(Arg...)</tt>.
///
/// @ingroup runtimeTypeTraits
//////////////////////////////////////////////////////////////////////
template<typename R, typename T, typename... Arg>
struct identity_member_pointer
{
  using type = R (T::*)(Arg...);
};


//////////////////////////////////////////////////////////////////////
/// @brief Returns a member function pointer of type
///        <tt>R (T::*)(Arg...) const</tt>.
///
/// @ingroup runtimeTypeTraits
//////////////////////////////////////////////////////////////////////
template<typename R, typename T, typename... Arg>
struct const_member_pointer
{
  using type = R (T::*)(Arg...) const;
};

} // namespace runtime


//////////////////////////////////////////////////////////////////////
/// @brief Returns an object used to do asynchronous RMIs to the member function
///        @p pmf of object @p h.
///
/// @warning An explicit call to @ref runtime::bind_rmi_result::flush() is
///          required to flush the requests.
///
/// @ingroup ARMIOneSided
///
/// @todo Allow binding the location ID.
/// @todo Allow binding any of the arguments, similarly to @c std::bind() and
///       @c std::placeholders.
/// @todo Allow avoiding to bind @p pmf and @p h.
//////////////////////////////////////////////////////////////////////
template<typename R, typename T, typename... Arg>
runtime::bind_rmi_result<R, T, runtime::identity_member_pointer, Arg...>
bind_rmi(R (T::*pmf)(Arg...), rmi_handle::reference const& h)
{
  using namespace runtime;
  return bind_rmi_result<R, T, identity_member_pointer, Arg...>{pmf, h};
}


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref bind_rmi for @c const member functions.
///
/// @ingroup ARMIOneSided
//////////////////////////////////////////////////////////////////////
template<typename R, typename T, typename... Arg>
runtime::bind_rmi_result<R, T, runtime::const_member_pointer, Arg...>
bind_rmi(R (T::*pmf)(Arg...) const, rmi_handle::const_reference const& h)
{
  using namespace runtime;
  return bind_rmi_result<R, T, const_member_pointer, Arg...>{pmf, h};
}

} // namespace stapl

#endif
