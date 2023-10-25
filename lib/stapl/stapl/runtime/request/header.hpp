/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_REQUEST_HEADER_HPP
#define STAPL_RUNTIME_REQUEST_HEADER_HPP

#include "../config.hpp"
#include "../context_id.hpp"
#include "../runtime_fwd.hpp"
#include "../utility/logical_clock.hpp"

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Provides the enum and the different headers for communication.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
class header
{
public:
  /// Header type.
  enum type
  {
    /// Uninitialized.
    INVALID = 0x0,

    /// Message with RPC requests.
    RPC,

    /// Message with location RPC requests.
    LOCATION_RPC,
    /// Forwarded message with location RPC requests.
    FWD_LOCATION_RPC,
    /// Broadcast message with location RPC requests.
    BCAST_LOCATION_RPC,

    /// Message with ordered RMI requests.
    RMI,
    /// Forwarded message with ordered RMI requests.
    FWD_RMI,
    /// Broadcast message with ordered RMI requests.
    BCAST_RMI,
    /// Forwarded broadcast message with ordered RMI requests.
    FWD_BCAST_RMI,
    /// Broadcast message with unordered RMI requests.
    UNORDERED_BCAST_RMI,
    /// Forwarded broadcast message with unordered RMI requests.
    FWD_UNORDERED_BCAST_RMI
  };

  using epoch_type = logical_clock::time_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Header for broadcast RPC to locations.
  //////////////////////////////////////////////////////////////////////
  class location_rpc final
  {
  private:
    const process_id    m_pid;
    const full_location m_dest;
    const epoch_type    m_epoch;

  public:
    location_rpc(full_location const& destination,
                 const epoch_type epoch) noexcept
    : m_pid(stapl::get_process_id()),
      m_dest(destination),
      m_epoch(epoch)
    { }

    process_id get_process_id(void) const noexcept
    { return m_pid; }

    full_location const& get_destination(void) const noexcept
    { return m_dest; }

    epoch_type get_epoch(void) const noexcept
    { return m_epoch; }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Header for broadcast RPCs to locations.
  //////////////////////////////////////////////////////////////////////
  class bcast_location_rpc final
  {
  private:
    const process_id m_pid;
    const gang_id    m_gid;
    const epoch_type m_epoch;

  public:
    bcast_location_rpc(const gang_id gid, const epoch_type epoch) noexcept
    : m_pid(stapl::get_process_id()),
      m_gid(gid),
      m_epoch(epoch)
    { }

    process_id get_process_id(void) const noexcept
    { return m_pid; }

    gang_id get_destination_gang_id(void) const noexcept
    { return m_gid; }

    epoch_type get_epoch(void) const noexcept
    { return m_epoch; }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Header for ordered and unordered point-to-point requests.
  //////////////////////////////////////////////////////////////////////
  class request final
  {
  private:
    const process_id m_pid;
    const context_id m_cid;
    const epoch_type m_epoch;

  public:
    request(context_id const& cid, const epoch_type epoch) noexcept
    : m_pid(stapl::get_process_id()),
      m_cid(cid),
      m_epoch(epoch)
    { }

    process_id get_process_id(void) const noexcept
    { return m_pid; }

    context_id const& get_context_id(void) const noexcept
    { return m_cid; }

    full_location const& get_destination(void) const noexcept
    { return m_cid.current; }

    epoch_type get_epoch(void) const noexcept
    { return m_epoch; }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Header for ordered and unordered broadcast requests.
  //////////////////////////////////////////////////////////////////////
  class bcast_request final
  {
  private:
    const process_id    m_pid;
    const full_location m_initiator;
    const gang_id       m_dst_gang_id;
    const full_location m_source;
    const bool          m_intragang;
    const nesting_level m_nesting;
    const magic_id      m_magic;
    const epoch_type    m_epoch;

  public:
    bcast_request(context_id const& cid, const epoch_type epoch) noexcept
    : m_pid(stapl::get_process_id()),
      m_initiator(cid.initiator),
      m_dst_gang_id(cid.current.get_gang_id()),
      m_source(cid.source),
      m_intragang(cid.intragang),
      m_nesting(cid.nesting),
      m_magic(cid.magic),
      m_epoch(epoch)
    { }

    bcast_request(full_location const& initiator,
                  const gang_id destination_gid,
                  full_location const& source,
                  const bool intragang,
                  const nesting_level nesting,
                  const epoch_type epoch) noexcept
    : m_pid(stapl::get_process_id()),
      m_initiator(initiator),
      m_dst_gang_id(destination_gid),
      m_source(source),
      m_intragang(intragang),
      m_nesting(nesting),
      m_magic(0),
      m_epoch(epoch)
    { }

    process_id get_process_id(void) const noexcept
    { return m_pid; }

    context_id make_context_id(const location_id lid) const noexcept
    {
      return context_id{m_initiator,
                        full_location{m_dst_gang_id, lid},
                        m_source,
                        m_intragang,
                        m_nesting,
                        m_magic};
    }

    gang_id get_destination_gang_id(void) const noexcept
    { return m_dst_gang_id; }

    epoch_type get_epoch(void) const noexcept
    { return m_epoch; }
  };
};

} // namespace runtime

} // namespace stapl

#endif
