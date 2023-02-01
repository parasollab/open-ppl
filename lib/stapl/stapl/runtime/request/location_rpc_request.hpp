/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_REQUEST_LOCATION_RPC_REQUEST_HPP
#define STAPL_RUNTIME_REQUEST_LOCATION_RPC_REQUEST_HPP

#include "../message.hpp"
#include "../exception.hpp"
#include <cstddef>

namespace stapl {

namespace runtime {

class location_md;


//////////////////////////////////////////////////////////////////////
/// @brief Encapsulates an RPC request directed to a location for subsequent
///        execution via the function operator.
///
/// @ref location_rpc_request objects package all information for buffering and
/// transfer.
///
/// The 'header' contains the size of the request. The 'body' (derived class)
/// has the desired function and any arguments required to invoke it.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
class location_rpc_request
{
public:
  using size_type = std::size_t;

private:
  size_type m_size;

public:
  constexpr explicit location_rpc_request(const size_type size) noexcept
  : m_size(size)
  { }

  location_rpc_request(location_rpc_request const&) = delete;
  location_rpc_request& operator=(location_rpc_request const&) = delete;

protected:
  ~location_rpc_request(void) = default;

public:
  constexpr size_type size(void) const noexcept
  { return m_size; }

  size_type& size(void) noexcept
  { return m_size; }

  virtual bool operator()(location_md&, message_shared_ptr&) = 0;
};


//////////////////////////////////////////////////////////////////////
/// @brief Executes RPC requests for a specific location, preserving the
///        requests that could not be executed for a later invocation.
///
/// @ingroup requestExecution
//////////////////////////////////////////////////////////////////////
class location_rpc_executor
{
private:
  message_shared_ptr          m_msg;
  message::payload_range_type m_payload;

public:
  explicit location_rpc_executor(message_ptr m) noexcept
  : m_msg(std::move(m)),
    m_payload(m_msg->payload())
  { }

  bool operator()(location_md& l)
  {
    STAPL_RUNTIME_ASSERT((m_msg->type()==header::LOCATION_RPC) ||
                         (m_msg->type()==header::BCAST_LOCATION_RPC));
    do {
      auto& req = *reinterpret_cast<location_rpc_request*>(m_payload.begin());
      const bool done = req(l, m_msg);
      if (!done)
        return false;
      m_payload.advance_begin(req.size());
    } while (!m_payload.empty());
    m_msg.reset();
    return true;
  }
};

} // namespace runtime

} // namespace stapl

#endif
