 
/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_REQUEST_HEADER_DECODER_HPP
#define STAPL_RUNTIME_REQUEST_HEADER_DECODER_HPP

#include "header.hpp"
#include "../context_id.hpp"
#include "../exception.hpp"
#include "../message.hpp"
#include "../runtime_fwd.hpp"

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Returns the epoch @p m was sent from.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
inline header::epoch_type get_message_epoch(message const& m) noexcept
{
  switch (m.type()) {
    case header::RMI:
      return m.get_extended_header<header::request>().get_epoch();

    case header::BCAST_RMI:
    case header::UNORDERED_BCAST_RMI:
      return m.get_extended_header<header::bcast_request>().get_epoch();

    default:
      STAPL_RUNTIME_ERROR("Unexpected request type.");
      return header::epoch_type{};
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns the context id, the epoch and the process id @p m was sent
///        from as a @c std::tuple.
///
/// @param lid Location id of the receiving location.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
inline std::tuple<context_id, header::epoch_type, process_id>
get_message_info(message const& m, const location_id lid) noexcept
{
  switch (m.type()) {
    case header::LOCATION_RPC: {
      auto const& h = m.get_extended_header<header::location_rpc>();
      return std::make_tuple(context_id{}, h.get_epoch(), h.get_process_id());
    }

    case header::BCAST_LOCATION_RPC: {
      auto const& h = m.get_extended_header<header::bcast_location_rpc>();
      return std::make_tuple(context_id{}, h.get_epoch(), h.get_process_id());
    }

    case header::RMI: {
      auto const& h = m.get_extended_header<header::request>();
      STAPL_RUNTIME_ASSERT(lid==h.get_context_id().current.get_location_id());
      return std::make_tuple(h.get_context_id(),
                             h.get_epoch(),
                             h.get_process_id());
    }

    case header::BCAST_RMI:
    case header::UNORDERED_BCAST_RMI: {
      auto const& h = m.get_extended_header<header::bcast_request>();
      return std::make_tuple(h.make_context_id(lid),
                             h.get_epoch(),
                             h.get_process_id());
    }

    default:
      STAPL_RUNTIME_ERROR("Unexpected request type.");
      return std::tuple<context_id, header::epoch_type, process_id>{};
  }
}

} // namespace runtime

} // namespace stapl

#endif
